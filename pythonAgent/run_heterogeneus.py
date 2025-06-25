import os
os.environ['JAX_ENABLE_X64'] = 'True'
import sys
import argparse
from collections import deque

import jax
import jax.numpy as jnp
import math
import numpy as np
from py_interface import *
from reinforced_lib import RLib
from reinforced_lib.agents.mab import *
from reinforced_lib.exts import BasicMab
from reinforced_lib.logs import *
from ns3_ai_structures import Env, Act


from ns3_ai_structures import Env, Act

MEMBLOCK_KEY = 2334
MEM_SIZE = 1024

N_CW = 10
N_RTS_CTS = 1
N_AMPDU = 1

ACTION_HISTORY_LEN = 20
ACTION_PROB_THRESHOLD = 0.9
LATENCY_THRESHOLD = 0.01

AGENT_ARGS = {
    'EGreedy': {
        'e': 1.0,
        'optimistic_start': 0.0,
        'e_min': 0.0,
        'e_decay': 0.985
    },
    'UCB': {
        'c': 1.0
    },
    'NormalThompsonSampling': {
        'alpha': 10.0,
        'beta': 0.2,
        'mu': 1.0,
        'lam': 0.0,
    }
}

if __name__ == '__main__':
    agent_name = "THR"
    thr = 100
    CHEATER_NUMBER = 10
    WIFI_NUMBER = 10

    logs_name = f"default_log.csv"
    csvPath_name = f"default_csv.csv"

    args = argparse.ArgumentParser()

    # global settings
    args.add_argument('--seed', type=int, default=4)
    args.add_argument('--mempoolKey', type=int, default=2333)
    args.add_argument('--ns3Path', type=str, default='')
    args.add_argument('--scenario', type=str, default='scenario_multi_agent')
    args.add_argument('--greedySta', type=int, default=0)
    args.add_argument('--blindlyFairSta', type=int, default=5)
    args.add_argument('--fullyFairSta', type=int, default=5)
    args.add_argument('--cautiousSta', type=int, default=0)
    args.add_argument('--legacySta', type=int, default=0)

    # ns-3 args
    args.add_argument('--agentName', type=str, default=agent_name)
    args.add_argument('--cheaterNumber', type=int, default=CHEATER_NUMBER)
    args.add_argument('--ampdu', action=argparse.BooleanOptionalAction, default=False)
    args.add_argument('--channelWidth', type=int, default=20)
    args.add_argument('--csvLogPath', type=str, default=logs_name)
    args.add_argument('--csvPath', type=str, default=csvPath_name)
    args.add_argument('--cw', type=int, default=-1)
    args.add_argument('--dataRate', type=int, default=thr)
    args.add_argument('--distance', type=float, default=10.0)
    args.add_argument('--flowmonPath', type=str, default='flowmon.xml')
    args.add_argument('--fuzzTime', type=float, default=5.0)
    args.add_argument('--interactionTime', type=float, default=0.5)
    args.add_argument('--interPacketInterval', type=float, default=0.5)
    args.add_argument('--maxQueueSize', type=int, default=100)
    args.add_argument('--mcs', type=int, default=11)
    args.add_argument('--nWifi', type=int, default=WIFI_NUMBER)
    args.add_argument('--packetSize', type=int, default=1500)
    args.add_argument('--rtsCts', action=argparse.BooleanOptionalAction, default=False)
    args.add_argument('--simulationTime', type=float, default=40.0)
    args.add_argument('--thrPath', type=str, default='thr.txt')
    args.add_argument('--cwMinDefault', type=int, default=32)
    args.add_argument('--cwMaxDefault', type=int, default=1024)

    # reward weights
    args.add_argument('--massive', type=float, default=0.0)
    args.add_argument('--throughput', type=float, default=1.0)
    args.add_argument('--urllc', type=float, default=0.0)

    # agent settings
    args.add_argument('--maxWarmup', type=int, default=50.0)
    args.add_argument('--useWarmup', action=argparse.BooleanOptionalAction, default=False)

    args = args.parse_args()
    args = vars(args)

    # read the arguments
    ns3_path = args.pop('ns3Path')

    if args['scenario'] == 'scenario_mgr_multi_agent':
        del args['interPacketInterval']
        del args['mcs']
        del args['thrPath']
        dataRate = min(115, args['dataRate'] * args['nWifi'])
    elif args['scenario'] == 'adhoc':
        del args['dataRate']
        del args['maxQueueSize']
        dataRate = (args['packetSize'] * args['nWifi'] / args['interPacketInterval']) / 1e6

    ns3_path = ""

    seed = args.pop('seed')
    key = jax.random.PRNGKey(seed)

    agent = args['agentName']
    nWifi = args['nWifi']
    interactionTime = args['interactionTime']
    mempool_key = args.pop('mempoolKey')
    scenario = args.pop('scenario')

    ns3_args = args
    ns3_args['RngRun'] = seed
    # set up the reward function
    reward_probs = np.asarray([args.pop('massive'), args.pop('throughput'), args.pop('urllc')])


    def reward_type_handler(greedySta, blindlyFairSta, fullyFairSta, cautiousSta, staNumber):
        if greedySta > staNumber:
            return "THR"
        elif greedySta + blindlyFairSta > staNumber:
            return "FAIR"
        elif greedySta + blindlyFairSta + fullyFairSta > staNumber:
            return "FULL_FAIR"
        elif greedySta + blindlyFairSta + fullyFairSta + cautiousSta > staNumber:
            return "COLLISION"


    def normalize_rewards(env, agent_num, rewardType):
        reward = 0
        if rewardType == "THR":
            throughput = env.throughput[agent_num] / dataRate
            reward = throughput
        elif rewardType == "FAIR":
            if env.time != 0:
                if env.tx_list[agent_num] != 0:
                    successRate = (env.rx_list[agent_num] / env.tx_list[agent_num])
                else:
                    successRate = 0.0
                reward = math.exp(-10 * abs(env.txTime[agent_num] * nWifi - (interactionTime)) ** 3)
                reward = (reward * successRate) - 1
        elif rewardType == "FULL_FAIR":
            if env.time != 0:
                reward = - abs(1 / nWifi - env.txTime[agent_num] / env.fullTxTime)
        elif rewardType == "SUCCESS":
            if env.time != 0:
                if env.tx_list[agent_num] != 0:
                    reward = (env.rx_list[agent_num] / env.tx_list[agent_num]) - 1
                else:
                    reward = -1
        elif rewardType == "EXP_FAIR":
            reward = -10 * (abs(1 / nWifi - env.txTime[agent_num] / env.fullTxTime)) / (
            (env.fullTxTime / interactionTime ** 2))
        elif rewardType == "COLLISION":
            if env.time != 0:
                if env.tx_list[agent_num] != 0:
                    reward = -((env.tx_list[agent_num] - env.rx_list[agent_num]) / env.tx_list[agent_num])
                else:
                    reward = -0.5
        else:
            print("UNKNOWN REWARD")

        rewards = np.asarray([reward])
        return np.dot(np.asarray([1]), rewards)


    # set up the warmup function
    max_warmup = args.pop('maxWarmup')
    use_warmup = args.pop('useWarmup')

    action_history = {
        'cw': deque(maxlen=ACTION_HISTORY_LEN),
    }

    def end_warmup(cw, time):
        if not use_warmup or time > max_warmup:
            return True

        action_history['cw'].append(cw)

        if len(action_history['cw']) < ACTION_HISTORY_LEN:
            return False

        max_prob = lambda actions: (np.unique(actions, return_counts=True)[1] / len(actions)).max()

        if min(max_prob(action_history['cw'])) > ACTION_PROB_THRESHOLD:
            return True

        return False


    # set up the agent
    if agent == 'wifi':
        rlib = None
    elif agent not in AGENT_ARGS:
        raise ValueError('Invalid agent type')
    else:
        rlib = RLib(
            agent_type=globals()[agent],
            agent_params=AGENT_ARGS[agent],
            ext_type=BasicMab,
            ext_params={'n_arms': N_CW},
            logger_types=CsvLogger,
            logger_params={'csv_path': f'rlib_{args["csvPath"]}'},
            logger_sources=('reward', SourceType.METRIC)
        )
        agent_id_list = []
        for i in range(args["cheaterNumber"]):
            agent_id_list.append(rlib.init(seed + i))

    # set up the environment
    exp = Experiment(mempool_key, MEM_SIZE, scenario, ns3_path, using_waf=False)
    var = Ns3AIRL(MEMBLOCK_KEY, Env, Act)

    try:
        # run the experiment
        ns3_process = exp.run(setting=ns3_args, show_output=True)

        while not var.isFinish():
            with var as data:
                if data is None:
                    break
                for i in range(args["cheaterNumber"]):
                    key, subkey = jax.random.split(key)
                    rewardType = reward_type_handler(greedySta=args['greedySta'], blindlyFairSta=args['blindlyFairSta'],
                                                     fullyFairSta=args['fullyFairSta'], cautiousSta=args['cautiousSta'],
                                                     staNumber=i)
                    reward = normalize_rewards(data.env, i, rewardType=rewardType)
                    action = rlib.sample(reward, agent_id=agent_id_list[i])  # dodac ID
                    cw = action
                    print(f"agent{i}: {action}")
                    rlib.log(f'cw{i}', cw)  # dodac ID
                    data.act.cw[i] = cw  # dodac ID
                    data.act.end_warmup = end_warmup(cw, data.env.time)

        ns3_process.wait()
    finally:
        del exp
        del rlib
