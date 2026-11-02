"""Login + origin protection for the Valkyrie Flask / Flask-SocketIO dashboard (drop-in, no other changes needed).

In main.py, right after  socketio = SocketIO(app, ...)  add:

    import hardening
    hardening.install(app, socketio, cfg, log=log_message)

What it does
  - Every page, video stream and Socket.IO connection needs a login first (a PIN / password).
  - The password comes from the VALKYRIE_PASSWORD environment variable, or config.json  "web": {"password": "..."},
    or (if neither is set) a random 6-digit PIN that is printed in the console on every start.
  - Wrong guesses are slowed down and locked out per IP address (5 failures -> 60 s).
  - Socket.IO connections from another website are refused (stops a web page you happen to open from arming or
    firing the turret over your network). Plain browsers on your own dashboard page are unaffected.
  - The session secret is random on every start instead of being fixed in the source code.
Optional config:  "web": {"password": "...", "max_failures": 5, "lockout_s": 60, "session_hours": 12}
Use it on a private network or your own hotspot. This is HTTP without encryption.
"""
import hmac
import os
import secrets
import time
from datetime import timedelta
from urllib.parse import urlparse

from flask import request, session, redirect, make_response

LOGIN_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Valkyrie C-UAV - sign in</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { margin: 0; min-height: 100vh; display: grid; place-items: center; background: #0b1220; color: #e6edf3;
         font: 16px/1.4 system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }
  .card { width: min(92vw, 380px); background: #131c2e; border: 1px solid #243049; border-radius: 14px; padding: 28px 24px; }
  h1 { font-size: 20px; margin: 0 0 4px; letter-spacing: .04em; }
  p { margin: 0 0 18px; color: #8b9bb4; font-size: 14px; }
  input { width: 100%; padding: 14px; border-radius: 10px; border: 1px solid #2b3a57; background: #0b1220; color: #e6edf3;
          font-size: 18px; letter-spacing: .15em; }
  input:focus { outline: 2px solid #2f81f7; border-color: transparent; }
  button { width: 100%; margin-top: 14px; padding: 14px; border: 0; border-radius: 10px; background: #2f81f7; color: #fff;
           font-size: 16px; font-weight: 600; cursor: pointer; }
  .err { margin-top: 12px; color: #ff7b72; font-size: 14px; min-height: 1.2em; }
</style></head><body>
<form class="card" method="post" action="/login">
  <h1>VALKYRIE C-UAV</h1>
  <p>Enter the access PIN shown in the turret console.</p>
  <input type="password" name="password" autocomplete="current-password" autofocus required placeholder="PIN / password">
  <input type="hidden" name="next" value="{next}">
  <button type="submit">Sign in</button>
  <div class="err">{error}</div>
</form></body></html>"""


def _safe_next(target):
    """Only allow redirects back to a path on this server."""
    if target and target.startswith("/") and not target.startswith("//"):
        return target
    return "/"


def install(app, socketio, cfg, log=print):
    web = cfg.get("web", {}) if isinstance(cfg, dict) else {}
    password = os.environ.get("VALKYRIE_PASSWORD") or str(web.get("password") or "")
    if not password:
        password = f"{secrets.randbelow(10 ** 6):06d}"
        log(f"[WEB] No password configured. PIN for this run: {password}")
    max_fail = int(web.get("max_failures", 5))
    lockout = float(web.get("lockout_s", 60))

    app.secret_key = secrets.token_hex(32)               # random per run, never stored in the source
    app.config.update(SESSION_COOKIE_HTTPONLY=True, SESSION_COOKIE_SAMESITE="Strict",
                      PERMANENT_SESSION_LIFETIME=timedelta(hours=float(web.get("session_hours", 12))))
    failures = {}                                         # ip -> [count, locked_until]

    def render(error="", nxt="/"):
        html = LOGIN_HTML.replace("{error}", error).replace("{next}", nxt.replace('"', ""))
        resp = make_response(html)
        resp.headers["Cache-Control"] = "no-store"
        return resp

    @app.route("/login", methods=["GET", "POST"], endpoint="valkyrie_login")
    def login():
        ip = request.remote_addr or "?"
        if request.method == "GET":
            return render(nxt=_safe_next(request.args.get("next", "/")))
        count, locked_until = failures.get(ip, [0, 0.0])
        if time.time() < locked_until:
            return render(f"Too many attempts. Try again in {int(locked_until - time.time())} s.",
                          _safe_next(request.form.get("next"))), 429
        given = request.form.get("password", "")
        if hmac.compare_digest(given.encode(), password.encode()):
            failures.pop(ip, None)
            session.clear()
            session["auth"] = True
            session.permanent = True
            log(f"[WEB] Sign-in from {ip}")
            return redirect(_safe_next(request.form.get("next")))
        count += 1
        failures[ip] = [count, time.time() + lockout if count >= max_fail else 0.0]
        log(f"[WEB] Failed sign-in from {ip} ({count})")
        time.sleep(1.0)                                   # slow down guessing
        return render("Wrong PIN or password.", _safe_next(request.form.get("next"))), 401

    @app.route("/logout", endpoint="valkyrie_logout")
    def logout():
        session.clear()
        return redirect("/login")

    @app.before_request
    def gate():
        if request.endpoint in ("valkyrie_login",):
            return None
        if session.get("auth") is True:
            return None
        if request.path.startswith(("/socket.io", "/video", "/stream", "/api", "/camera")) or request.is_json:
            return ("Unauthorized", 401)
        return redirect("/login?next=" + request.path)

    def on_connect(auth=None):
        """Socket.IO connect: must be signed in, and must come from this site (not another web page)."""
        if session.get("auth") is not True:
            log("[WEB] Socket.IO connection refused: not signed in")
            return False
        origin = request.headers.get("Origin")
        if origin and urlparse(origin).netloc != request.host:
            log(f"[WEB] Socket.IO connection refused: foreign origin {origin}")
            return False
        return None

    socketio.on_event("connect", on_connect)
    return on_connect
