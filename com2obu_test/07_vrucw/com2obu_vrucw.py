#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import run_rsm,set_participant
JSON=Path(__file__).with_name("vrucw.json")
EGO=[(30.4636068,114.1243498),(30.4629615,114.1229769)]
PED=(30.4631546,114.1233891)
def update(data,t,ego,i,n):
    data["value"]["msgCnt"]=i%128
    # 行人直接放在主车前方路径上，先验证 OBU 的 VRUCW 通讯/算法触发
    set_participant(data["value"]["participants"][0],PED[0],PED[1],0.0,0.0,3)
if __name__=="__main__":
    print("[VRUCW] pedestrian on ego path; expected category=VRUCW event_sort=10 ptcType=3")
    run_rsm(JSON,"983966a4-59a2-4431-bd62-965d7ec8ee27",EGO,30.0,16.0,update,2)
