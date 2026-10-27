#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import run_rsm,set_participant,distance_m,interpolate
JSON=Path(__file__).with_name("fcw.json")
EGO=[(30.4636068,114.1243498),(30.4629615,114.1229769)]
T0=(30.4632836,114.1236637); TEND=(30.4629615,114.1229769); TOTAL=distance_m(T0,TEND)
def update(data,t,ego,i,n):
    data["value"]["msgCnt"]=i%128
    # 前车约10km/h，主车50km/h，从后方快速追近，构造 TTC 下降
    ratio=min(0.95,((10.0/3.6)*t)/TOTAL); p=interpolate(T0,TEND,ratio)
    set_participant(data["value"]["participants"][0],p[0],p[1],10.0,241.5,1)
if __name__=="__main__":
    print("[FCW] ego=50km/h target=10km/h; expected category=fFCW event_sort=0")
    run_rsm(JSON,"983966a4-59a2-4431-bd62-965d7ec8ee26",EGO,50.0,10.0,update,3)
