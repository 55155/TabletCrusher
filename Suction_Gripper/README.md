# Suction Gripper

## 구성
| 부품 | 모델 | 자료 |
| --- | --- | --- |
| 스텝모터 (그리퍼 부근) | NK112810H (Nema 11, 28각) | `docs/NK112810H_spec.jpg` |
| 모터 드라이버 | SBC-11R4 | `docs/SBC-11R4_protocol_manual.pdf` |
| 진공 발생기 (솔밸브 일체형) | SMC ZM101H-K5LZ | `docs/SMC_ZM_catalog_EU.pdf` |
| 석션 탱크 | Bittle_bug 4 bar | 자료 미확보 |

## NK112810H 스펙
| 항목 | 값 |
| --- | --- |
| 스텝각 | 1.8° (2상) |
| 모터 길이 | 28 mm |
| 정격 전류 | 0.8 A |
| 상 저항 / 인덕턴스 | 3.2 Ω / 1.7 mH |
| 홀딩 토크 | 8 N·cm (0.08 N·m) |
| 로터 관성 | 7 g·cm² |
| 리드선 | 4 (AWG26) |
| 중량 | 90 g |
| 플랜지 / 축 | □28 mm (Nema 11), 축 Ø5 mm, M2.5 탭 4개 |

결선(도면 표기): A·Ā = RED·GRN, B·B̄ = RED·BLU.
도면에 RED가 두 번 표기되어 있으므로 실물 리드선 색은 테스터로 확인할 것.

## SMC ZM101H-K5LZ
- ZM 시리즈: 흡입 필터·사일렌서·공급 밸브·파기 밸브·진공 스위치가 일체형인 진공 이젝터
- 형번 해석과 사양은 `docs/SMC_ZM_catalog_EU.pdf`의 How to Order 참고

## TODO
- Bittle_bug 석션 탱크 카탈로그 확보 (제조사/모델 미확인)
- SMC ZM 취급설명서(TF2Z371EN) 원본 링크 404 → 별도 확보
