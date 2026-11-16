"""Independently check exported fit inputs, display curves and truth metrics."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parent


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output",nargs="?",type=Path,default=ROOT/"outputs/regression_fits")
    args=parser.parse_args();out=args.output
    manifest=json.loads((out/"manifest.json").read_text())
    audit=json.loads((out/"audit.json").read_text())
    assert sha(ROOT/"visualize_regression.py")==audit["exporter_sha256"]
    for name,digest in manifest["source_implementation_sha256"].items():assert sha(ROOT/name)==digest
    max_coeff_error=0.;max_metric_error=0.;combined=[];frame_inputs={}
    for entry in audit["cases"]:
        path=out/"cases"/(entry["case"]+".json");assert sha(path)==entry["sha256"]
        d=json.loads(path.read_text());scene,count,frame=d["scene"],d["count"],d["frame"]
        points=np.array(d["points"]).reshape(-1,3);retained=np.array(d["inducing"]).reshape(-1,3)
        ids=np.array(d["inducing_ids"])
        assert len(points)==d["partition"]["input"] and len(retained)==count and len(np.unique(ids))==count
        np.testing.assert_allclose(points[ids],retained,atol=1.01e-7,rtol=0)
        if scene=="flat_ramp":
            scan_path=out/f"flat_ramp_scan_{frame:03d}.npz"
            assert sha(scan_path)==d["source"]["scan_sha256"]
            assert sha(out/"flat_ramp_clean.npz")==d["source"]["clean_points_sha256"]
            fitted=out/f"flat_ramp_m{count}_{frame:03d}.npz"
        else:
            scan_path=ROOT.parent/d["source"]["evidence_path"]
            assert sha(scan_path)==d["source"]["evidence_sha256"]
            run=ROOT/("outputs/table_cpu" if scene=="table" else "outputs/cpu")
            fitted=run/f"{scene}_m{count}_fsgp.npz"
            assert sha(fitted)==d["source"]["saved_gp_sha256"]
        with np.load(scan_path) as data:original=data["points"]
        np.testing.assert_allclose(points,original,atol=5.1e-8,rtol=0)
        key=(scene,frame)
        if key in frame_inputs:assert sha(scan_path)==frame_inputs[key],"M variants used different input"
        else:frame_inputs[key]=sha(scan_path)
        with np.load(fitted) as data:coarse=data["xy"];saved_retained=data["inducing"][:,:3]
        np.testing.assert_allclose(original[ids],saved_retained,atol=1e-12,rtol=0)
        x=np.linspace(coarse[:,0].min(),coarse[:,0].max(),101)
        y=np.linspace(coarse[:,1].min(),coarse[:,1].max(),101)
        np.testing.assert_allclose(d["x"],x,atol=5.1e-8,rtol=0);np.testing.assert_allclose(d["y"],y,atol=5.1e-8,rtol=0)
        gx,gy=np.meshgrid(x,y)
        values={k:np.array([np.nan if z is None else z for z in d[field]]) for k,field in
                [("gp","gp_height"),("local","local_height"),("global","quadratic_height")]}
        assert all(v.shape==(10201,) for v in values.values())
        a,b,c,e,f,g=d["quadratic"]["coefficients"];ox,oy=d["quadratic"]["origin"]
        u,v=gx.ravel()-ox,gy.ravel()-oy
        explicit=a*u*u+b*u*v+c*v*v+e*u+f*v+g
        good=np.isfinite(values["global"])
        coefficient_error=float(np.max(np.abs(explicit[good]-values["global"][good])))
        assert coefficient_error<1e-5
        max_coeff_error=max(max_coeff_error,coefficient_error)
        if scene!="flat_ramp":continue
        truth=.3*np.clip(gx.ravel()+1,0,2)
        np.testing.assert_allclose(d["truth"],truth,atol=5.1e-8,rtol=0)
        common=np.logical_and.reduce([np.isfinite(v) for v in values.values()])
        xx=gx.ravel()
        regions={"all":common,"seams":common&(np.minimum(np.abs(xx+1),np.abs(xx-1))<=.2),
                 "flat_low":common&(xx<-1.2),"ramp":common&(np.abs(xx)<.8),"flat_high":common&(xx>1.2)}
        for region,mask in regions.items():
            for method,prediction in values.items():
                error=prediction[mask]-truth[mask]
                computed={"n":int(mask.sum()),"mae_m":float(np.mean(np.abs(error))),
                          "rmse_m":float(np.sqrt(np.mean(error**2))),"p95_m":float(np.percentile(np.abs(error),95))}
                expected=d["analytic_errors"][region][method]
                assert computed["n"]==expected["n"]
                delta=max(abs(computed[k]-expected[k]) for k in ["mae_m","rmse_m","p95_m"])
                assert delta<1e-6,(path,region,method,delta)
                max_metric_error=max(max_metric_error,delta)
                combined.append({"count":count,"region":region,"method":method,**computed})
    summary=json.loads((out/"combined_summary.json").read_text())
    for expected in summary:
        items=[r for r in combined if all(r[k]==expected[k] for k in ["count","region","method"])]
        n=sum(r["n"] for r in items);assert n==expected["n"]
        mae=sum(r["mae_m"]*r["n"] for r in items)/n
        rmse=np.sqrt(sum(r["rmse_m"]**2*r["n"] for r in items)/n)
        assert abs(mae-expected["mae_m"])<1e-6 and abs(rmse-expected["rmse_m"])<1e-6
    report={"passed":True,"cases":len(audit["cases"]),"shared_input_frames":len(frame_inputs),
            "combined_region_method_metrics_recomputed":len(combined),"pooled_rows_recomputed":len(summary),
            "max_serialized_polynomial_error_m":max_coeff_error,"max_metric_difference_m":max_metric_error,
            "checks":["Exporter and original mapping source hashes match","Case hashes and original NPZ inputs match",
                "M variants share the same XYZ input","Displayed retained points match the actual fitted observations",
                "All global quadratic displays match their explicit polynomial",
                "All composite truth values, region metrics and 8-frame pooled metrics independently recomputed"]}
    (out/"independent_audit.json").write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=="__main__":main()
