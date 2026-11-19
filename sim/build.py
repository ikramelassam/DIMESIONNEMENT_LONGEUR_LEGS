"""Assemble sim/gr2_simulateur.html : interface + modele JS + entrees de config.yaml."""
import json, os, yaml
H = os.path.dirname(os.path.abspath(__file__))
ui = open(os.path.join(H, "ui.html"), encoding="utf-8").read()
cmpjs = open(os.path.join(H, "compare.js"), encoding="utf-8").read()
model = open(os.path.join(H, "model.js"), encoding="utf-8").read()
cfg = yaml.safe_load(open(os.path.join(os.path.dirname(H), "config.yaml"), encoding="utf-8"))
cfg["gabarit"] = 400.0
cfg.setdefault("gabarit_min", 0.0)
val = json.load(open(os.path.join(H, "validation.json")))
vo = json.load(open(os.path.join(H, "validation_opt.json")))
vp = json.load(open(os.path.join(H, "validation_pipe.json"))) if os.path.exists(os.path.join(H, "validation_pipe.json")) else None
rf = open(os.path.join(H, "ref_fronts.json"), encoding="utf-8").read()
va = open(os.path.join(H, "validation_audit.json"), encoding="utf-8").read() if os.path.exists(os.path.join(H, "validation_audit.json")) else "null"
opt = open(os.path.join(H, "opt.js"), encoding="utf-8").read()
assert "</script" not in opt
assert "</script" not in model
out = ui.replace("/*CMP*/", cmpjs).replace("/*MODEL*/", model).replace("/*CONFIG*/", json.dumps(cfg, ensure_ascii=False)).replace("/*VALIDATION*/", json.dumps(val)).replace("/*VALIDATION_OPT*/", json.dumps(vo)).replace("/*VALIDATION_PIPE*/", json.dumps(vp)).replace("/*REF_FRONTS*/", rf).replace("/*VALIDATION_AUDIT*/", va).replace("/*OPT*/", opt)
open(os.path.join(H, "gr2_simulateur.html"), "w", encoding="utf-8").write(out)
print("ok", round(len(out) / 1024), "Ko")
