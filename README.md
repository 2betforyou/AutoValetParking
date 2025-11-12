# 🚗 Autonomous Valet Parking System
> 중앙 제어형 자율 발렛 주차 시스템 (CAU 2025 AI·SW 융합 우수성과 발표대회 출품작)

---

## 📖 프로젝트 개요
**Autonomous Valet Parking System**은 실내 주차장 환경에서도 차량이 스스로 경로를 탐색하고,  
중앙 제어 서버가 모든 차량의 위치와 이동을 실시간으로 관리하여 **충돌 없는 입출차와 공간 효율성 극대화**를 실현하는 시스템입니다.

- **문제 인식**  
  - 출구 근처만 이용하려는 운전자 패턴으로 인한 병목 및 공간 낭비  
  - 차량 간 충돌 위험 및 불필요한 대기시간 발생  
  - 실내 주차장에서 GPS가 작동하지 않아 위치 인식 불가  

- **해결 접근**  
  - 중앙 제어 서버 기반의 **ReservationTable + Network Graph(A\*) 경로 제어**  
  - **SimPy 이벤트 시뮬레이션 엔진**을 이용한 시간·공간 기반 최적화  
  - SLAM / GCSLAM 기반 차량 위치 추정 및 자율 주행  

---

## 🧠 주요 기능
| 구분 | 기능 설명 |
|------|------------|
| 🕹️ 중앙 제어 서버 | 모든 차량의 위치·경로·층간 이동을 실시간 관리하며, 동일 도로 구간의 중복 예약을 방지 |
| 🚘 차량 에이전트 | GCSLAM 기반 센서 융합을 통해 실내 환경에서도 자율 위치 추정 및 이동 수행 |
| ⏱️ ReservationTable | 시간 단위로 각 노드·엣지를 예약하여 교차 충돌 방지 및 교통 흐름 동기화 |
| 🅿️ 스팟 보호 시스템 | SimPy `Resource` 객체를 이용해 주차 스팟 점유/선점 상태를 동시 제어 |
| 🧭 지능형 경로 탐색 | A\* 알고리즘 기반 최단·최적 경로 탐색 (교착 구간 자동 회피) |

---

## ⚙️ 시스템 구성
- **Simulation Core** : `SimPy` 기반의 이벤트 시뮬레이션
- **Control Server** : Python 모듈 기반 중앙 통제 로직
- **Vehicle Agent** : SLAM 모듈 시뮬레이션
- **Graph Planner** : A\* + ReservationTable 조합의 시간 기반 경로 탐색기
- **Output Visualization** : Matplotlib / ASCII 맵 기반 주차장 시각화

---

## 💻 실행 방법
### 1️⃣ 환경 세팅
```bash
git clone https://github.com/<username>/<repo_name>.git
cd <repo_name>
pip install -r requirements.txt

---

## 📄 라이선스

본 프로젝트는 교육 및 연구 목적의 비상업적 사용에 한합니다.
상업적 사용 또는 2차 배포 시 반드시 팀의 허가가 필요합니다.

---


