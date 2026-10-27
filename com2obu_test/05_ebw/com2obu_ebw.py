#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import run_rsm,set_participant,distance_m,interpolate
JSON=Path(__file__).with_name("ebw.json")
EGO=[(30.4636068,114.1243498),(30.4629615,114.1229769)]
T0=(30.4633482,114.1238010); TEND=(30.4629615,114.1229769); TOTAL=distance_m(T0,TEND)
def update(data,t,ego,i,n):
    data["value"]["msgCnt"]=i%128
    # 前车先以36km/h正常行驶3秒，随后突然停车，制造紧急制动状态变化
    moving=min(t,3.0); ratio=min(0.95,(10.0*moving)/TOTAL); p=interpolate(T0,TEND,ratio)
    target_speed=36.0 if t<3.0 else 0.0
    set_participant(data["value"]["participants"][0],p[0],p[1],target_speed,241.5,1)
if __name__=="__main__":
    print("[EBW] lead vehicle 36km/h -> 0km/h at t=3s; expected category=EBW event_sort=5")
    run_rsm(JSON,"983966a4-59a2-4431-bd62-965d7ec8ee25",EGO,36.0,12.0,update,3)
