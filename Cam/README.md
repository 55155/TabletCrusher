# Cam (현미경 카메라)

## 모델
- 제품: OMAX 2MP USB 2.0 C-Mount Microscope Camera
- 센서/OEM: ToupTek `UCMOS02000KPB`
- USB ID: `VID_0547&PID_6201`
- 전원: USB 버스 전원 (별도 전원 불필요)

## 스펙 (OMAX A3520U 제품 정보)
| 항목 | 값 |
| --- | --- |
| 모델 | A3520U |
| 센서 | Aptina color CMOS |
| 해상도 | 1600×1200 (2MP) |
| 픽셀 크기 | 3.2 µm × 3.2 µm |
| SNR / Dynamic Range | 43 dB / 61 dB |
| 프레임 속도 | 16 fps @1600×1200, 50 fps @800×600 |
| 인터페이스 / 전원 | USB 2.0 / USB 버스 전원 |
| 마운트 | C-mount 또는 아이피스 튜브 (23.2 / 30 / 30.5 mm) |
| 축소 렌즈 | 0.5X, 23.2 mm 아이피스 튜브용 |
| 캘리브레이션 슬라이드 | 0.01 mm (1 mm / 100 눈금, stage micrometer) |
| 크기 | 63 mm × 37 mm (축소 렌즈 제외) |
| 동작 온도 | -10 ~ 50 °C |
| 총중량 | 0.62 kg |

구성품: 카메라, 0.5X 축소 렌즈, 23.2→30 mm·23.2→30.5 mm 어댑터,
0.01 mm 캘리브레이션 슬라이드, USB 케이블, CD(드라이버·소프트웨어·설명서)

## 드라이버 / 소프트웨어
- 드라이버: ToupTek `toupcam.inf` / `toupcam.sys` (서명: HANGZHOU TOUPTEK PHOTONICS)
- 소프트웨어: ToupView (OMAX판 `OMAXToupViewSetup_20250112`, ToupTek판 4.12.31488)
  - ToupTek 공식: https://www.touptekphotonics.com/ko/download/
- **현재 상태: 장치 오류 39 (드라이버 로드 차단)**
  - 원인: `toupcam.sys`가 Windows 메모리 무결성(HVCI)과 비호환
    (CodeIntegrity 이벤트 3111, "not compatible with hypervisor enforcement")
  - OMAX판(1.0.0.27501)·ToupTek판(1.0.0.30959)의 `toupcam.sys`는 해시 동일 → 드라이버 교체로는 해결 불가
  - 해결: 메모리 무결성 해제 후 재부팅, 또는 HVCI가 꺼진 다른 PC 사용
- 연결: 허브 경유 시 Descriptor Request Failed 발생 이력 → 본체 후면 포트에 직결

## 작업: 파쇄 후 입도(PSD) 측정
목적은 `docs/Paper.md` §3-5의 go/no-go 확인이다. 스트로크 수에 따라 d50이
어디서 평탄해지는지 확인하고, 그 결과로 XRD 단계 진행 여부를 정한다.
`docs/PINN.md`의 PBM/PINN 학습에 쓰는 스트로크별 PSD 데이터도 여기서 만든다.

- 입력: 스트로크 수별 파쇄 시료 현미경 이미지
- 출력: 스트로크 수별 PSD, d50
- 입도 구간: `docs/PINN.md` 기준 √2 체열 850/600/425/300/212/150/106/75/53/0 µm (9구간)
- 스트로크 3~4점 이상, <75 µm 구간 정밀 (`docs/PINN.md` 측정 계획)

### TODO
1. 카메라 인식 (위 드라이버 문제 해결)
2. 동봉 0.01 mm 슬라이드로 µm/px 환산 (배율·축소 렌즈 조합마다)
3. 캡처 스크립트 (ToupTek SDK)
4. 입자 분할 → 등가 직경 → 구간별 질량/개수 분포 → d50
