# Isaac Sim (애커먼) 시뮬레이터 연결

설치부터 실행까지: [docs/IsaacSim_준비_안내.md](../../docs/IsaacSim_준비_안내.md)

- `limo_ros2_ackermann/` Isaac Sim 4.5 확장 (WeGo LimoIsaacSIM 폴더 안에 링크해서 사용)
- `tools/ack_stamp_relay.py` /limo/ack_cmd → /sim/ackermann_cmd (Stamped)
- `tools/dxf_to_usd.py` 대회 도면 DXF → 바닥 트랙 USD
- `launch/isaac_sim.launch.py` 중계 + 카메라 압축 + 변환 노드 + 우리 노드

Isaac Sim 안에서 실행 확인은 아직 안 했습니다(작성 환경에 Isaac Sim 없음).
