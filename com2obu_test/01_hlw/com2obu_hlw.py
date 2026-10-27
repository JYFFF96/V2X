#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import run_rsi
if __name__=="__main__":
    run_rsi(Path(__file__).with_name("hlw.json"),"rte","983966a4-59a2-4431-bd62-965d7ec8ee04",36.0,3)
