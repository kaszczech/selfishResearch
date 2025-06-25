from ctypes import *


# ns3-ai environment structure
class Env(Structure):
    _pack_ = 1
    _fields_ = [
        ('fairness', c_double),
        ('latency', c_double),
        ('plr', c_double),
        ('time', c_double),
        ('fullTxTime', c_double),
        ('rx_list', c_double * 10),
        ('lost_list', c_double * 10),
        ('throughput', c_double * 10),
        ('tx_list', c_double * 10),
        ('txTime', c_double * 10),
    ]


# ns3-ai action structure
class Act(Structure):
    _pack_ = 1
    _fields_ = [
        ('end_warmup', c_bool),
        ('cw', c_int * 10)
    ]

