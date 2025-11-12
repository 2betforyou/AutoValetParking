# autovp.py
# -*- coding: utf-8 -*-
"""
중앙 관제(Control Tower) + 시공간 예약(Reservation) + A* 경로탐색
ASCII 맵을 '셀 단위 격자 그래프'로 변환:
 - 통행 가능 셀: '0', 'G', 'R', '-', '|'
 - 통행 불가 셀: ' '(공백)
규칙:
 - 'G' : 주차 가능
 - 'R' : 이미 점유(초기 점유)
 - '0' : 교차/입구(층별 entrance로 사용)
"""

import simpy
import random
import networkx as nx
from collections import defaultdict

# -----------------------------
# 시뮬레이션 파라미터
# -----------------------------
random.seed(42)

EDGE_TRAVEL_TIME = 2.0        # 엣지(셀 간) 통과 시간(초)
ARRIVAL_GAP       = 12.0      # 차량 도착 간격(초)
REPLAN_DELAY      = 3.0       # 예약 실패 시 재시도(초)
PARK_TIME_RANGE   = (60, 180) # 쇼핑 체류시간(초)
SIM_TIME          = 240.0     # 전체 시뮬레이션 시간(초)

FLOOR_CATEGORY = {"B1": "food", "B2": "fashion"}

# -----------------------------
# 예약 테이블 (시공간 엣지 예약)
# -----------------------------
class ReservationTable:
    def __init__(self):
        self.book = defaultdict(list)  # edge_key -> [(t0, t1, vid)]

    @staticmethod
    def norm_edge(u, v):
        return tuple(sorted((u, v)))

    def can_reserve(self, edge_key, t0, t1):
        for (a, b, _) in self.book[edge_key]:
            if not (t1 <= a or b <= t0):  # 시간이 겹치면 불가
                return False
        return True

    def reserve_path(self, path_edges, start_t, vid):
        t = start_t
        temp = []
        for (u, v) in path_edges:
            ek = self.norm_edge(u, v)
            t0, t1 = t, t + EDGE_TRAVEL_TIME
            if self.can_reserve(ek, t0, t1):
                temp.append((ek, t0, t1))
                t = t1
            else:
                return None
        for ek, t0, t1 in temp:
            self.book[ek].append((t0, t1, vid))
        return t

# -----------------------------
# ASCII → 격자 그래프 (셀 단위)
# -----------------------------
TRAV = set("0GR-|")  # 통행 가능 셀

def grid_graph_from_ascii(lines):
    """
    각 (i,j) 셀을 노드로 하고, 상하좌우 인접 셀 중 통행 가능한 경우 엣지 생성.
    노드 ID는 f"C{i}_{j}"(Cell)로 통일.
    """
    H = len(lines)
    W = max(len(r) for r in lines)
    # 각 줄 폭을 동일하게 맞추기
    lines = [r + " " * (W - len(r)) for r in lines]

    G = nx.Graph()
    def cid(i, j): return f"C{i}_{j}"

    for i in range(H):
        for j in range(W):
            c = lines[i][j]
            if c in TRAV:
                G.add_node(cid(i, j), ch=c, ij=(i, j))
    # 상하좌우 연결
    for i in range(H):
        for j in range(W):
            if lines[i][j] not in TRAV:
                continue
            u = cid(i, j)
            # 우
            if j + 1 < W and lines[i][j+1] in TRAV:
                G.add_edge(u, cid(i, j+1))
            # 하
            if i + 1 < H and lines[i+1][j] in TRAV:
                G.add_edge(u, cid(i+1, j))
    return G

def find_cells(G, want):
    """특정 문자(want: '0','G','R','-','|')를 가진 셀 목록 반환"""
    return [n for n, d in G.nodes(data=True) if d.get("ch") == want]

# -----------------------------
# 경로 탐색 (A*)
# -----------------------------
def astar_path(G, s, t):
    def xy(n):
        i, j = G.nodes[n]["ij"]
        return i, j
    def h(a, b):
        ax, ay = xy(a); bx, by = xy(b)
        return abs(ax - bx) + abs(ay - by)
    try:
        return nx.astar_path(G, s, t, heuristic=h, weight=None)
    except nx.NetworkXNoPath:
        return None

def to_edges(path_nodes):
    return list(zip(path_nodes[:-1], path_nodes[1:]))

# -----------------------------
# 중앙 관제 (Control Tower)
# -----------------------------
class ControlTower:
    def __init__(self, env, floors):
        self.env = env
        self.floors = floors            # {"B1": (G, entrance_cell), ...}
        self.resv = ReservationTable()
        # 초기 점유: 'R' 셀을 점유로 표기
        self.occupied = set()           # {(fid, cell_id)}
        for fid, (G, _entr) in floors.items():
            for rcell in find_cells(G, "R"):
                self.occupied.add((fid, rcell))

    def log(self, msg):
        print(f"[T={self.env.now:6.1f}] [TOWER] {msg}")

    def spot_candidates(self, fid):
        G, _ = self.floors[fid]
        return [c for c in find_cells(G, "G") if (fid, c) not in self.occupied]

    def pick_spot_by_preference(self, pref):
        """
        소비패턴 기반 스팟 선택(아주 단순):
         - 층 가중치: B1→food, B2→fashion
         - 각 층 entrance에서 가까운 G 선호
        """
        best = None; best_score = -1e18
        for fid, (G, entrance) in self.floors.items():
            cat = FLOOR_CATEGORY[fid]; w = pref.get(cat, 0.0)
            for sp in self.spot_candidates(fid):
                p = astar_path(G, entrance, sp)
                if not p:
                    continue
                dist = len(p) - 1
                score = w * (-dist) + 0.1
                if score > best_score:
                    best_score = score
                    best = (fid, sp, dist)
        return best

    def reserve_path(self, fid, path_nodes, vid):
        G, _ = self.floors[fid]
        edges = to_edges(path_nodes)
        return self.resv.reserve_path(edges, self.env.now, vid)

# -----------------------------
# 차량 에이전트
# -----------------------------
class Vehicle:
    def __init__(self, env, vid, tower, pref):
        self.env = env; self.vid = vid; self.tower = tower; self.pref = pref
        env.process(self.run())

    def log(self, msg):
        print(f"[T={self.env.now:6.1f}] car#{self.vid} {msg}")

    def run(self):
        # 1) 스팟 선정
        choice = self.tower.pick_spot_by_preference(self.pref)
        if not choice:
            self.log("배정 가능한 스팟이 없습니다. 종료")
            return
        fid, spot, _ = choice
        G, entrance = self.tower.floors[fid]
        self.log(f"배정 층={fid}, 입구={entrance}, 스팟={spot}")

        # 2) 입차 경로 예약/이동
        cur = entrance
        while True:
            path = astar_path(G, cur, spot)
            if not path:
                self.log("입차 경로 없음 → 재시도")
                yield self.env.timeout(REPLAN_DELAY); continue
            if self.tower.reserve_path(fid, path, self.vid) is None:
                self.log("입차 예약 충돌 → 대기")
                yield self.env.timeout(REPLAN_DELAY); continue
            travel_t = (len(path)-1) * EDGE_TRAVEL_TIME
            self.log(f"입차 이동 시작 (예상 {travel_t:.1f}s)")
            yield self.env.timeout(travel_t)
            break

        # 3) 점유
        self.tower.occupied.add((fid, spot))
        self.log("주차 완료(점유)")

        # 4) 체류
        dwell = random.randint(*PARK_TIME_RANGE)
        self.log(f"체류 시작 ({dwell}s)")
        yield self.env.timeout(dwell)

        # 5) 출차
        self.log("출차 요청")
        while True:
            path = astar_path(G, spot, entrance)
            if not path:
                self.log("출차 경로 없음 → 재시도")
                yield self.env.timeout(REPLAN_DELAY); continue
            if self.tower.reserve_path(fid, path, self.vid) is None:
                self.log("출차 예약 충돌 → 대기")
                yield self.env.timeout(REPLAN_DELAY); continue
            travel_t = (len(path)-1) * EDGE_TRAVEL_TIME
            self.log(f"출차 이동 시작 (예상 {travel_t:.1f}s)")
            yield self.env.timeout(travel_t)
            break

        # 6) 점유 해제
        self.tower.occupied.discard((fid, spot))
        self.log("출구 도착 / 점유 해제")

# -----------------------------
# 차량 도착 프로세스
# -----------------------------
def arrival_process(env, tower):
    vid = 0
    while True:
        vid += 1
        # 홀수차=식품 선호, 짝수차=패션 선호
        pref = {"food": 0.7, "fashion": 0.3} if vid % 2 == 1 else {"food": 0.3, "fashion": 0.7}
        Vehicle(env, vid, tower, pref)
        yield env.timeout(ARRIVAL_GAP)

# -----------------------------
# 층 그래프 생성 (ASCII → 격자 그래프)
# -----------------------------
ASCII_MAP = [
    "0-----0",
    "|-G  R-|",
    "|-G  G-|",
    "|-R  G-|",
    "|-G  R-|",
    "0-----0",
]

def build_floors():
    # 각 층은 동일 형상을 쓰되 entrance 다르게 지정(예시)
    G1 = grid_graph_from_ascii(ASCII_MAP)
    G2 = grid_graph_from_ascii(ASCII_MAP)
    # entrance: 맨 윗줄 좌측 '0' / 우측 '0' 선택
    # 좌측 0 찾기
    zeros1 = find_cells(G1, "0")
    # 위쪽 행 0들 중 가장 왼쪽: 단순히 (i,j) 최소로
    def pick_leftmost_zero(G):
        zs = find_cells(G, "0")
        zs_ij = [(n, G.nodes[n]["ij"]) for n in zs]
        zs_ij.sort(key=lambda x: (x[1][0], x[1][1]))
        return zs_ij[0][0] if zs_ij else None
    def pick_rightmost_zero(G):
        zs = find_cells(G, "0")
        zs_ij = [(n, G.nodes[n]["ij"]) for n in zs]
        zs_ij.sort(key=lambda x: (x[1][0], -x[1][1]))
        return zs_ij[0][0] if zs_ij else None

    ent_B1 = pick_leftmost_zero(G1)   # "N0_0"과 유사
    ent_B2 = pick_rightmost_zero(G2)  # "N0_6"과 유사

    floors = {
        "B1": (G1, ent_B1),
        "B2": (G2, ent_B2),
    }
    return floors

# -----------------------------
# 실행부
# -----------------------------
def main():
    env = simpy.Environment()
    floors = build_floors()
    tower = ControlTower(env, floors)

    # 초기 점유 로그
    init_r = sum(1 for x in tower.occupied)
    tower.log(f"초기 점유(R) 스팟 개수={init_r}")

    env.process(arrival_process(env, tower))
    print("=== Parking Simulation Start ===")
    env.run(until=SIM_TIME)
    print("=== Simulation End ===")

if __name__ == "__main__":
    main()