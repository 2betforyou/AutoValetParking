# conda activate /dev/8tb_2/envs/autopk

import simpy
import random

# --- 1. 시뮬레이션 상수 정의 ---
RANDOM_SEED = 42
RAMP_CYCLE_TIME = 30  # 램프 신호 주기 (초)
RAMP_CLEAR_TIME = 5   # 램프가 비워지는 시간 (초)
DRIVE_TO_RAMP_TIME = (1, 5) # 램프까지 가는 데 걸리는 시간 (min, max)
DRIVE_ON_RAMP_TIME = 10 # 램프를 통과하는 데 걸리는 시간
CAR_ARRIVAL_RATE = 2  # X초마다 1대씩 차 도착
CUSTOMER_SHOPPING_TIME = (60, 180) # 고객 쇼핑 시간 (min, max)

# --- 2. 중앙 관제 서버 (ControlTower) ---
class ControlTower:
    """
    중앙 관제 서버 클래스.
    램프 접근을 제어하는 로직을 포함합니다.
    """
    def __init__(self, env):
        self.env = env
        self.up_queue = simpy.Store(env)
        self.down_queue = simpy.Store(env)
        self.env.process(self.ramp_controller())

    def ramp_controller(self):
        """
        [핵심 로직] 램프를 '시분할'로 제어하는 스마트 신호등.
        30초는 상행, 30초는 하행을 처리합니다.
        """
        print(f"[T=0] 🚦 램프 제어 시스템 가동.")
        
        while True:
            # --- 1. 하행 사이클 (30초간) ---
            print(f"[T={self.env.now:.0f}] 🚦 램프 '하행(⬇️)' 전용 (30초간)")
            cycle_end = self.env.now + RAMP_CYCLE_TIME

            while self.env.now < cycle_end:
                remaining_time = cycle_end - self.env.now
                # get 이벤트와 timeout 이벤트를 경쟁시킴
                get_evt = self.down_queue.get()
                to_evt  = self.env.timeout(remaining_time)
                res = yield (get_evt | to_evt)

                if get_evt in res:
                    car = res[get_evt]
                    print(f"[T={self.env.now:.0f}] 🚗 Car {car.car_id} (하행) 램프 진입 승인.")
                    self.env.process(car.drive_on_ramp("하행"))
                else:
                    # 남은 시간이 끝나 창구 종료
                    break

            print(f"[T={self.env.now:.0f}] 🚦 램프 정리 시간 ({RAMP_CLEAR_TIME}초)")
            yield self.env.timeout(RAMP_CLEAR_TIME)
            
            # --- 2. 상행 사이클 (30초간) ---
            print(f"[T={self.env.now:.0f}] 🚦 램프 '상행(⬆️)' 전용 (30초간)")
            cycle_end = self.env.now + RAMP_CYCLE_TIME

            while self.env.now < cycle_end:
                remaining_time = cycle_end - self.env.now
                get_evt = self.up_queue.get()
                to_evt  = self.env.timeout(remaining_time)
                res = yield (get_evt | to_evt)

                if get_evt in res:
                    car = res[get_evt]
                    print(f"[T={self.env.now:.0f}] 🚙 Car {car.car_id} (상행) 램프 진입 승인.")
                    self.env.process(car.drive_on_ramp("상행"))
                else:
                    break

            print(f"[T={self.env.now:.0f}] 🚦 램프 정리 시간 ({RAMP_CLEAR_TIME}초)")
            yield self.env.timeout(RAMP_CLEAR_TIME)

# --- 3. 차량 프로세스 (AVP 차량) ---
class Car:
    """
    자율주행 차량 1대를 나타내는 클래스입니다.
    SimPy 프로세스로 작동합니다.
    """
    def __init__(self, env, car_id, tower):
        self.env = env
        self.car_id = car_id
        self.tower = tower
        self.action = env.process(self.run())
        self.ramp_pass_event = env.event()

    def run(self):
        # --- 1. 입차 (상행) ---
        print(f"[T={self.env.now:.0f}] 🚙 Car {self.car_id} (상행) 1층 도착. 램프 대기.")
        yield self.env.timeout(random.randint(*DRIVE_TO_RAMP_TIME))
        yield self.tower.up_queue.put(self)
        yield self.ramp_pass_event
        print(f"[T={self.env.now:.0f}] 🚙 Car {self.car_id} (상행) 4층 주차 완료.")
        
        # --- 2. 쇼핑 ---
        shopping_time = random.randint(*CUSTOMER_SHOPPING_TIME)
        yield self.env.timeout(shopping_time)
        
        # --- 3. 출차 (하행) ---
        print(f"[T={self.env.now:.0f}] 🚗 Car {self.car_id} (하행) 4층 키오스크 호출. 램프 대기.")
        self.ramp_pass_event = self.env.event()
        yield self.env.timeout(random.randint(*DRIVE_TO_RAMP_TIME))
        yield self.tower.down_queue.put(self)
        yield self.ramp_pass_event
        print(f"[T={self.env.now:.0f}] 🚗 Car {self.car_id} (하행) 1층 출구로 나감.")

    def drive_on_ramp(self, direction):
        """램프 컨트롤러에 의해 호출되는 함수"""
        yield self.env.timeout(DRIVE_ON_RAMP_TIME)
        self.ramp_pass_event.succeed()

# --- 4. 시뮬레이션 실행 ---
def setup_simulation(env):
    tower = ControlTower(env)
    car_id = 0
    while True:
        car_id += 1
        Car(env, car_id, tower)
        yield env.timeout(CAR_ARRIVAL_RATE)

random.seed(RANDOM_SEED)
env = simpy.Environment()
env.process(setup_simulation(env))

print("--- AVP 램프 통제 시뮬레이션 시작 ---")
env.run(until=500)