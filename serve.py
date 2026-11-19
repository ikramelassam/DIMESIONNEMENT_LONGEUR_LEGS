"""Serveur local : relie le simulateur HTML au code Python COMPLET d'optimisation.

    python serve.py            ->  http://localhost:8765

Le simulateur detecte ce serveur (/api/ping) et propose alors le moteur
"Python complet" (memes calculs que run_exact.py : numpy vectorise, 8192 points
de Sobol, raffinement, Monte-Carlo...). Sans serveur, le simulateur utilise son
port JavaScript du meme pipeline (resolution reduite par defaut).
Aucune dependance hors bibliotheque standard + celles du programme."""
import json, math, os, sys, threading, time, traceback, uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import yaml
from gr2.params import Params
from gr2 import pipeline, explore as ex

PAGE = os.path.join(HERE, "sim", "gr2_simulateur.html")
JOBS = {}
LOCK = threading.Lock()
COLS = ["L1", "L2", "L3", "alpha", "beta", "xi", "D", "H2", "h", "servo_C1", "servo_C2", "servo_C3", "f1_masse_pattes", "f2_nesm",
        "f3_couple_C2", "frac_bloc_C1", "frac_bloc_C2", "frac_bloc_C3", "tau_C1", "tau_C2", "tau_C3", "tau_min_C1", "tau_min_C2",
        "tau_min_C3", "span_C1", "span_C2", "span_C3", "M_total", "phase_critique", "taux_reussite", "ssm_corr_min"]


def clean(o):
    if isinstance(o, dict): return {k: clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)): return [clean(v) for v in o]
    if isinstance(o, (np.floating, float)): return None if not math.isfinite(float(o)) else float(o)
    if isinstance(o, (np.integer,)): return int(o)
    if isinstance(o, np.bool_): return bool(o)
    return o


def rows(df, cap=None):
    if df is None or len(df) == 0: return []
    d = df
    if cap and len(d) > cap:
        d = d.iloc[np.linspace(0, len(d) - 1, cap).astype(int)]
    d = d[[c for c in COLS if c in d.columns]]
    return clean(json.loads(d.to_json(orient="records")))


def load_default():
    with open(os.path.join(HERE, "config.yaml"), encoding="utf-8") as f:
        return yaml.safe_load(f)


def run_job(job, body):
    try:
        kind = body.get("kind", "optimize")
        raw = pipeline.deep_merge(load_default(), body.get("p") or {})
        raw.pop("chassis_mount", None)
        mount = body.get("mount") or raw["chassis"]["mount"]
        sc = body.get("sc", "S2")
        log = lambda m: job["log"].append(f"[{time.strftime('%H:%M:%S')}] {m}")
        if kind == "screen":
            P = Params.from_dict({**raw, "chassis": {**raw["chassis"], "mount": mount}})
            job["result"] = clean(pipeline.screen_servos(P, sc, n=int(body.get("n", 512)), log=log))
        else:
            out = pipeline.optimize(raw, mount, sc, allowed=body.get("allowed"), n_sobol=body.get("n_sobol"),
                                    refine_rounds=body.get("refine_rounds"), refine_per_point=body.get("refine_per_point"),
                                    n_mc=body.get("n_mc"), seed=body.get("seed"), log=log,
                                    stop=lambda: job["stop"], progress=lambda f: job.__setitem__("progress", f))
            job["result"] = clean(dict(res=out["res"], adm=rows(out["df"], 3000), front=rows(out["front"]), cand=rows(out["rob"]),
                                       rob_front=rows(out["rob_front"])))
        job["status"] = "stopped" if job["stop"] else "done"
    except Exception as e:
        job["status"] = "error"; job["error"] = f"{e}"; job["trace"] = traceback.format_exc()
    job["progress"] = 1.0 if job["status"] == "done" else job["progress"]
    job["t1"] = time.time()


class H(BaseHTTPRequestHandler):
    def _send(self, code, data, ctype="application/json"):
        b = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(b)))
        self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Access-Control-Allow-Headers", "content-type")
        self.end_headers(); self.wfile.write(b)

    def log_message(self, *a): pass

    def do_OPTIONS(self): self._send(204, b"")

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            if not os.path.exists(PAGE): return self._send(404, {"error": "construire sim/gr2_simulateur.html : python sim/build.py"})
            return self._send(200, open(PAGE, "rb").read(), "text/html; charset=utf-8")
        if path == "/api/ping": return self._send(200, {"ok": True, "engine": "python", "cores": os.cpu_count(), "numpy": np.__version__})
        if path == "/api/config": return self._send(200, clean(load_default()))
        if path.startswith("/api/job/"):
            j = JOBS.get(path.split("/")[3])
            if not j: return self._send(404, {"error": "job inconnu"})
            out = {k: j.get(k) for k in ("status", "progress", "log", "error", "trace")}
            out["elapsed"] = round((j.get("t1") or time.time()) - j["t0"], 1)
            if j["status"] in ("done", "stopped"): out["result"] = j["result"]
            return self._send(200, out)
        self._send(404, {"error": "inconnu"})

    def do_POST(self):
        path = self.path.split("?")[0]
        n = int(self.headers.get("Content-Length", 0)); body = json.loads(self.rfile.read(n) or b"{}")
        if path == "/api/job":
            jid = uuid.uuid4().hex[:10]
            job = dict(status="running", progress=0.0, log=[], stop=False, t0=time.time(), result=None)
            with LOCK: JOBS[jid] = job
            threading.Thread(target=run_job, args=(job, body), daemon=True).start()
            return self._send(200, {"id": jid})
        if path.startswith("/api/job/") and path.endswith("/stop"):
            j = JOBS.get(path.split("/")[3])
            if j: j["stop"] = True
            return self._send(200, {"ok": True})
        self._send(404, {"error": "inconnu"})


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765
    print(f"Simulateur + optimisation Python : http://localhost:{port}   (Ctrl+C pour arreter)")
    ThreadingHTTPServer(("127.0.0.1", port), H).serve_forever()
