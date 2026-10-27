#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import run_rsm,set_participant
JSON=Path(__file__).with_name("avw.json")
EGO=[(30.4636068,114.1243498),(30.4629615,114.1229769)]
TARGET=(30.4631546,114.1233891)
def update(data,t,ego,i,n):
    data["value"]["msgCnt"]=i%128
    set_participant(data["value"]["participants"][0],TARGET[0],TARGET[1],0.0,241.5,1)
if __name__=="__main__":
    print("[AVW] stationary vehicle ahead; expected event2hmi category=AVW event_sort=6")
    run_rsm(JSON,"983966a4-59a2-4431-bd62-965d7ec8ee24",EGO,36.0,14.0,update,2)
