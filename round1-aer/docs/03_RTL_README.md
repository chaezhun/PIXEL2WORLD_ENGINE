# RTL / 시뮬레이션 README

Bio-mimic Neuron을 위한 AER 통신 방식 설계 — 소스 구성과 실행 방법

---

## 1. 파일 구성

### `rtl/` — 설계

**기본 소자**

| 파일 | 역할 |
|---|---|
| `mutex.v` | 2-input 상호배제 소자 (arbiter cell). Seitz(1980) |
| `c_element.v` | Muller C-element. behavioural / Earle-latch structural |
| `delay_chain.v` | bundled-data matched delay |

**인코더 (micropipeline 계열)**

| 파일 | 역할 |
|---|---|
| `aer_node.v` | 2-input arbitered encoder node. semi-decoupled 이중 C-element |
| `aer_tree.v` | 임의 깊이 바이너리 트리 (재귀 인스턴스화). `DEPTH` 파라미터 |
| `aer_tree8.v` | 8-event 고정 트리. `aer_tree #(.DEPTH(3))` 의 회귀 검증용 |

**readout 구조 (비교 대상)**

| 파일 | 대응 | 역할 |
|---|---|---|
| `arb_tree.v` | — | level-mode arbiter tree, one-hot grant (전통 AER 방식) |
| `aer_rowcol.v` | **B1 / B2** | row/column readout. 두 포맷이 이 회로를 공유 |
| `aer_scan.v` | **B3** | 동기 scan + global hold/reset (Gen3 계열) |
| `aer_hybrid.v` | **제안** | 부하 연동 스냅샷 타임스탬핑 + row/column arbiter |

### `sim/` — 테스트벤치

| 파일 | 대상 | 비고 |
|---|---|---|
| `tb_aer_tree.v` | `aer_tree` | `DEPTH` 스윕용. 스케일링 법칙 측정 |
| `tb_aer_tree8.v` | `aer_tree8` | 8채널 상세 계측 (노드별 래치/전달 프로브) |
| `tb_aer_rowcol.v` | `aer_rowcol` | 버스트 길이 → word-serial 비트/이벤트 |
| `tb_aer_scan.v` | `aer_scan` | readout latency와 timestamp error를 **분리 측정** |
| `tb_aer_hybrid.v` | `aer_hybrid` | 제안 구조 + 베이스라인 (동일 TB), 공간 클러스터링 자극 |

---

## 2. 구현 스위치 — behavioural / structural

세 기본 소자가 두 가지 구현을 갖는다. `define`으로 선택한다.

| 매크로 | 대상 | structural 구현 |
|---|---|---|
| `AER_STRUCT_C` | `c_element` | Earle latch (3 AND + 1 OR). 실제 조합 루프 |
| `AER_STRUCT_MUTEX` | `mutex` | cross-coupled NAND + 준안정 필터 |
| `AER_STRUCT_DLY` | `delay_chain` | dont_touch 버퍼 체인 |

- **define 없음 = behavioural.** 시뮬레이션 기본값이며 결정론적이다.
- **define 있음 = structural.** 합성 대상 경로이며, 조합 루프 때문에 DRC 예외가 필요하다
  (`LUTLP-1` severity 완화 또는 `ALLOW_COMBINATORIAL_LOOPS`).

각 테스트벤치는 실행 시작 시 **BUILD 배너**로 어느 경로가 컴파일됐는지 출력한다.
세 줄이 모두 `STRUCTURAL`이어야 structural 빌드다.

```
===== BUILD =====
 c_element   : STRUCTURAL
 mutex       : STRUCTURAL, window = 0.040 ns
 delay_chain : STRUCTURAL
 margin check: window 0.040 ns > C-element 0.020 ns -> ok
=================
```

---

## 3. 주요 파라미터

| 파라미터 | 위치 | 의미 | 기본값 |
|---|---|---|---|
| `DEPTH` | `aer_tree` | 채널 수 = 2^DEPTH | 3 |
| `ROW_D` / `COL_D` | `aer_rowcol`, `aer_scan`, `aer_hybrid` | 배열 크기 = 2^ROW_D × 2^COL_D | 3, 3 |
| `REQ_DLY` | 전체 | `r_req` matched delay 단수. 타이밍 클로저의 주 노브 | 4 |
| `C_TPD` | → `c_element.TPD_NS` | C-element 출력지연 [ns]. **0이면 안 된다** | 0.020 |
| `M_TPD` | → `mutex.TPD_NS` | mutex 기본 게이트 지연 [ns] | 0.020 |
| `T_CLK` | `aer_scan` | 위치당 스캔 주기 [ns] | 1.86 |
| `TH_ROWS` | `aer_hybrid` | 스냅샷 트리거 활성 행 수. **> NROW 이면 기능 비활성** | 4 |
| `T_SETTLE` | `aer_hybrid` | 입력 집합 재구성 후 arbiter 정착 대기 [ns] | 1.0 |
| `CLUSTER` | `tb_aer_hybrid` | 자극 공간 상관: 0 iid / 1 가로 / 2 세로 / 3 블록 | 0 |
| `CL_SIZE` | `tb_aer_hybrid` | 클러스터 픽셀 수 | 4 |

### 반드시 지켜야 할 제약

```
mutex 핸드오버 창  >  C-element 응답시간
  structural :  2.0 × M_TPD  >  C_TPD
  behavioural:  2.5 × M_TPD  >  C_TPD
```

기본값(50 ps 또는 40 ps 대 20 ps)은 2~2.5배 여유를 갖는다. `C_TPD`를 올려 이 부등식이 깨지는 지점을 찾으면 그것이 **측정된 타이밍 마진**이다.

`C_TPD = 0`은 금지다. `cin → cout → ~cout → cin` 이 zero-delay 피드백 루프가 되어 주소 flip-flop이 미확정 값으로 클록될 수 있다.

---

## 4. 실행 방법 (Vivado / XSim)

### 4.1 프로젝트 준비

```tcl
add_files -norecurse <경로>/rtl
add_files -fileset sim_1 -norecurse <경로>/sim
set_property -name {xsim.simulate.runtime} -value {-all} -objects [get_filesets sim_1]
```

> **⚠️ `xsim.simulate.runtime` 을 반드시 `-all` 로 설정할 것.**
> 기본값 1000 ns 로는 첫 단계도 끝나지 않는다.

### 4.2 테스트벤치 선택과 파라미터

`generic` 은 **테스트벤치를 바꿔도 fileset에 남는다.** 파라미터 이름이 다르면 elaboration이 실패하므로 전환할 때마다 다시 설정해야 한다.

```tcl
# 트리 스케일링
set_property top tb_aer_tree [get_filesets sim_1]
set_property generic {DEPTH=6} [get_filesets sim_1]

# row/column
set_property top tb_aer_rowcol [get_filesets sim_1]
set_property generic {ROW_D=3 COL_D=3} [get_filesets sim_1]

# 제안 구조 (TH_ROWS=9 이면 베이스라인)
set_property top tb_aer_hybrid [get_filesets sim_1]
set_property generic {ROW_D=3 COL_D=3 TH_ROWS=4 CLUSTER=0 CL_SIZE=4} [get_filesets sim_1]
```

### 4.3 structural 빌드

```tcl
set_property verilog_define {AER_STRUCT_C AER_STRUCT_MUTEX AER_STRUCT_DLY} [get_filesets sim_1]
# 되돌리기
set_property verilog_define {} [get_filesets sim_1]
```

> `Settings → General → Verilog options → Defines` 는 합성(`sources_1`)용이므로 시뮬레이션에는 적용되지 않을 수 있다. 위 Tcl 또는 `Settings → Simulation → Compilation` 의 `xsim.compile.xvlog.more_options` 를 사용할 것.

### 4.4 배치 실행

```tcl
proc run_cfg {gen} {
  set_property generic $gen [get_filesets sim_1]
  for {set try 0} {$try < 3} {incr try} {
    if {![catch {launch_simulation} msg]} { close_sim; return 1 }
    puts "  -- retry [expr {$try+1}]"
    catch {close_sim}
  }
  return 0
}

foreach cfg { {TH_ROWS=9 CLUSTER=0} {TH_ROWS=4 CLUSTER=0} \
              {TH_ROWS=9 CLUSTER=1} {TH_ROWS=4 CLUSTER=1} } {
  puts "########## $cfg ##########"
  run_cfg "ROW_D=3 COL_D=3 CL_SIZE=4 $cfg"
}
```

> Vivado가 `[Common 17-180] Spawn failed` 로 첫 실행에 실패하는 경우가 있다. 로그에 **VRFC 오류가 없고** `'compile' step finished` 가 있으면 설계 문제가 아니라 프로세스 생성 실패이며, 재시도로 통과한다. 위 `catch` + 재시도 패턴을 권장한다.

---

## 5. 테스트벤치 구조 — 6단계

| PHASE | 내용 | 판정 기준 |
|---|---|---|
| **0 IDLE** | 자극 없이 대기 | 출력이 있으면 자유발진 |
| **1 DIRECTED** | 채널을 하나씩 순차 발화 | 주소 인코딩 정확성 |
| **2 BURST** | 전 채널 동시 발화 | 전부 배출되어야 함. grant 교대 경합 검출 |
| **3 POISSON** | 단일 부하 통계 | 보존 검사, in-flight ≤ 노드 수 |
| **4 SWEEP** | 1 → 1000 Meps 로그 스윕 | 용량 곡선, timestamp error |
| **5 FAIRNESS** | 동기 버스트 반복 | 서비스 순위 편향 |

### 자극 모델

- **Poisson 도착** + **per-source one-slot gating**: 픽셀은 pending 요청을 1개만 보유하며, 초과분은 중앙 큐가 아니라 **소스에서** 손실된다.
- **공간 클러스터링** (`tb_aer_hybrid`): 실제 DVS 이벤트는 엣지를 따라 뭉치므로, iid Poisson만으로는 버스트 압축을 과소평가한다. `CLUSTER` 로 가로/세로/블록 상관을 선택한다.

### 계측

| 지표 | 설명 |
|---|---|
| throughput / loss | 완료 이벤트율, 소스 측 손실률 |
| readout latency | mean / min / **worst** |
| **timestamp error** | 타임스탬프 시각 − 발생 시각. arbiter에서는 readout latency와 같지만 scan·제안 구조에서는 다르다 |
| `ts_bet` / `ts_wit` | 타임스탬프 오차의 일원배치 분산분석 (채널 간 / 채널 내) |
| `kbar` | 행 grant당 이벤트 수 → word-serial 비트/이벤트 |
| `hold%` | 스냅샷이 걸린 시간 비율 |
| `n_cin` / `n_ack` / `n_out` | 배출 경로 3자 계수 — 래치 / acknowledge / 배출. **셋이 같아야 정상** |
| `ORPH` / `PHAN` / `SPUR` | 픽셀별 ack·출력 정합성 (0이어야 정상) |
| `BAD LATCH` | 선택된 픽셀 없이 주소가 래치되면 상태를 출력. **절대 나오면 안 됨** |

### 검증 자산

테스트벤치에는 스스로 오류를 신고하는 검사가 들어 있다. 회귀 시험 시 아래가 모두 성립해야 한다.

| 검사 | 조건 |
|---|---|
| 배출 경로 | `n_cin = n_ack = n_out` |
| 픽셀 정합성 | `ORPH = PHAN = SPUR = 0` |
| 보존 | 단계별 래치 합계 = accepted, in-flight ≤ 노드 수 |
| 주소 | `BAD LATCH` 출력 없음, `ERR = 0` |

### 로그 읽는 법

- Vivado는 launch 시 1000 ns를 먼저 자동 실행한 뒤 `run -all` 을 에코한다. 따라서 **PHASE 0~2 결과가 `run -all` 위로 올라갈 수 있다.**
- **맨 끝 VERDICT 블록**을 보면 된다.

```
============ VERDICT ============
   PHASE 0 IDLE     : PASS
   PHASE 1 DIRECTED : PASS
   PHASE 2 BURST    : PASS
   PHASE 3 POISSON  : PASS
   OVERALL : PASS
=================================
```

---

## 6. 알려진 사항

| 항목 | 내용 |
|---|---|
| **재귀 인스턴스화 경고** | `aer_tree`·`arb_tree` 는 자기 자신을 인스턴스화한다. Vivado 계층 분석기가 `Circular Reference Found` 경고를 내지만 정상이며, elaboration은 `DEPTH=1` 에서 종료된다 |
| **behavioural 경로는 시뮬 전용** | `assign #(...)` 과 primitive delay는 합성에서 무시된다. 합성은 structural 경로로 해야 한다 |
| **`GNT_GAP` 파라미터** | 실제 지연 소자에서만 의미가 있다. behavioural `delay_chain` 은 lumped inertial delay이므로 넓히면 핸드오버 펄스를 삼킨다 |
| **재구성 시점 제약** | `aer_hybrid` 의 제어부는 입력 집합을 바꾸기 전에 ① 정착 게이트 하강 ② C-element 전파지연 대기 ③ `cin`·`cout` 이 모두 0이 될 때까지 대기 를 순서대로 수행한다. **`cin == 0` 만으로 유휴를 판정하면 안 된다** — C-element는 입력에서 래치하고 유지하므로 출력이 아직 0이어도 트랜잭션이 이미 확정되어 있을 수 있다. 이 순서를 무너뜨리면 선택된 픽셀 없이 주소가 배출된다 |

---

## 7. 재현 순서 (권장)

```
1. tb_aer_tree,   DEPTH = 3,4,5,6,7        → 스케일링 법칙 (τ₀, Δ, 용량)
2. tb_aer_rowcol, ROW_D=COL_D=3            → row/column 베이스라인, 버스트 길이
3. tb_aer_scan,   ROW_D=COL_D=3            → scan 베이스라인, 타임스탬프 분리 효과
4. tb_aer_hybrid, TH_ROWS=9 / 4            → 베이스라인 vs 제안 (동일 자극)
5. tb_aer_hybrid, CLUSTER=0,1,2,3          → 공간 상관이 압축에 미치는 영향
6. structural define 추가 후 1~4 재실행    → 합성 대상 경로 검증
```
