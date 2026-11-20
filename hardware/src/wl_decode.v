// wl_decode : MGROUP word 스트림 -> 변환 파이프라인으로 보낼 job.
//
// 입력 word 형식(bit 31 이 MSB 이고, 종류는 맨 위 바이트에 있다):
//   event word   [31] = 1.
//                [7:0]   아래 bitmap: bit k 가 1 이면 행 8*G + k 에 이벤트가 있다
//                [15:8]  위 bitmap: bit k 가 1 이면 행 8*(G + A) + k 에 이벤트가 있다
//                [17:16] 아래 / 위 그룹의 극성. 쓰지 않는다: 월드 메모리는
//                        두 극성을 똑같이 센다
//                [25:18] G, 아래 8행 그룹 번호(720 행이면 0 .. 89 라
//                        [24:18] 만 읽는다)
//                [30:26] A, 위 그룹 번호에서 아래 그룹 번호를 뺀 값(0 .. 31).
//                        A = 0 이면 두 bitmap 이 같은 8 행을 덮는다.
//   column addr  [31:24] = 0x04, [10:0] 열 x(0 .. 959 라 [9:0] 만 읽는다).
//                뒤따르는 event word 는 이 열에 속한다.
//   frame end    [31:24] = 0x0c, [18:11] 8비트 frame counter.
//   timestamp    [31:24] = 0x08, header [31:24] = 0x7f: 둘 다 무시한다.
//
// job(클록마다 많아야 하나, 스트림 순서대로. j_type / j_val / j_map):
//   COL x        : 열 x 가 현재 열이다(새 열 주소이거나, 새 frame 이 시작될 때
//                  같은 열을 다시 낸 것).
//   GRP y0, map  : 비어 있지 않은 8행 그룹 하나. y0 = 8 * group, map = 8비트 행 bitmap.
//                  event word 하나는 비어 있지 않은 bitmap 마다 GRP job 을 하나씩 낸다(아래
//                  먼저, 그다음 위).
//   BND          : frame 경계. 다음 frame 은 다음 phase 를 쓴다.
//
// frame 경계 규칙(파서 software/dvs_develop/dvs_read.py 와 같다): end marker 는 그
// counter 가 직전 end marker 의 counter 와 다를 때만 경계다. 리셋 뒤 첫 end marker 는
// 그 앞에 event 나 column word 가 있었을 때만 경계다:
//   - 원본 녹화는 frame 0 중간에서 시작하므로 첫 end marker 가 frame 0 을
//     닫는다(파서와 같다);
//   - frame f0-1 의 end marker 로 시작하는 시험 벡터는 counter 만 잡아 두고,
//     그다음 frame f0 이 첫 phase 를 쓴다.
// 빈 frame 0 으로 시작하는 스트림(어떤 event 보다 end marker 가 먼저)은 frame 을
// 하나 모자라게 센다.
//
// x 항 C*(x-cx) 는 frame 의 각도에 따라 달라지므로, 경계 뒤에는 현재 열을 다시
// 곱해야 한다. 그 COL job 은 미뤄 두었다가 다음 event word 앞에서 낸다(x_stale). 그래서
// 스트림 끝의 경계는 파이프라인을 빈 채로 남긴다.
// 열 주소보다 먼저 온 event word 는 파서처럼 x = 0 을 쓴다.
//
// 1023 보다 위의 행은 나타낼 수 없다: G + A 는 128 로 나눈 나머지로 잡힌다. 센서는
// 720 행이라(G + A <= 89) 올바른 입력에서는 이런 일이 없다.
//
// handshake: word 하나를 그 job 을 모두 내보낼 때까지 잡아 둔다(wv, wd).
// in_ready 는 잡아 두는 레지스터가 비어 있거나 이번 클록에 비워질 때 1 이다.
module wl_decode (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] in_word,
    input  wire        in_valid,
    output wire        in_ready,

    output reg         j_valid,
    input  wire        j_ready,
    output reg   [1:0] j_type,
    output reg   [9:0] j_val,      // COL: x, GRP: y0
    output reg   [7:0] j_map,      // GRP: 행 bitmap

    output wire        busy,
    output reg  [15:0] frame_cnt   // 리셋 뒤 본 경계 수
);
    localparam [1:0] J_COL = 2'd0, J_GRP = 2'd1, J_BND = 2'd2;

    // 잡아 둔 word 와 거기서 아직 낼 것.
    reg        wv;
    reg [31:0] wd;
    reg        p_lo, p_hi;          // event word: 아직 낼 아래 / 위 그룹

    reg        x_stale;             // 파이프라인의 Xu / Xv 가 (x, frame) 과 맞지 않는다
    reg  [9:0] x_cur;
    reg        seen_end;
    reg        seen_data;           // event 나 column word 를 처리한 적이 있다
    reg  [7:0] cnt;

    wire       is_ev  = wd[31];
    wire       is_col = !wd[31] && (wd[31:24] == 8'h04);
    wire       is_end = !wd[31] && (wd[31:24] == 8'h0c);
    wire [6:0] g_lo   = wd[24:18];
    wire [6:0] g_hi   = wd[24:18] + {2'b00, wd[30:26]};

    // 잡아 둔 word 가 이번 사이클에 할 일(out_free 일 때만 실행한다). event word 는
    // 열이 stale 이면 먼저 열을 다시 내고, 그다음 아래 그룹, 그다음 위 그룹을 낸다.
    // 하나에 한 클록씩 걸린다.
    wire       out_free = !j_valid || j_ready;
    wire       ev_col   = is_ev && x_stale;
    wire       ev_lo    = is_ev && !x_stale && p_lo;
    wire       ev_hi    = is_ev && !x_stale && !p_lo && p_hi;
    wire       bnd      = is_end && (seen_end ? (wd[18:11] != cnt) : seen_data);
    // 이번 사이클의 job 을 끝으로 word 처리가 끝난다.
    wire       w_last   = !is_ev || (!x_stale && (p_lo ? !p_hi : 1'b1));
    wire       w_done   = wv && out_free && w_last;

    assign in_ready = !wv || w_done;
    assign busy     = wv || j_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wv        <= 1'b0;
            wd        <= 32'd0;
            p_lo      <= 1'b0;
            p_hi      <= 1'b0;
            x_stale   <= 1'b1;
            x_cur     <= 10'd0;
            seen_end  <= 1'b0;
            seen_data <= 1'b0;
            cnt       <= 8'd0;
            j_valid   <= 1'b0;
            j_type    <= J_COL;
            j_val     <= 10'd0;
            j_map     <= 8'd0;
            frame_cnt <= 16'd0;
        end else begin
            if (out_free) begin
                j_valid <= 1'b0;
                if (wv) begin
                    if (is_ev || is_col) seen_data <= 1'b1;
                    if (ev_col) begin
                        j_valid <= 1'b1;
                        j_type  <= J_COL;
                        j_val   <= x_cur;
                        x_stale <= 1'b0;
                    end else if (ev_lo) begin
                        j_valid <= 1'b1;
                        j_type  <= J_GRP;
                        j_val   <= {g_lo, 3'b000};
                        j_map   <= wd[7:0];
                        p_lo    <= 1'b0;
                    end else if (ev_hi) begin
                        j_valid <= 1'b1;
                        j_type  <= J_GRP;
                        j_val   <= {g_hi, 3'b000};
                        j_map   <= wd[15:8];
                        p_hi    <= 1'b0;
                    end else if (is_col) begin
                        j_valid <= 1'b1;
                        j_type  <= J_COL;
                        j_val   <= wd[9:0];
                        x_cur   <= wd[9:0];
                        x_stale <= 1'b0;
                    end else if (is_end) begin
                        if (bnd) begin
                            j_valid   <= 1'b1;
                            j_type    <= J_BND;
                            x_stale   <= 1'b1;
                            frame_cnt <= frame_cnt + 16'd1;
                        end
                        seen_end <= 1'b1;
                        cnt      <= wd[18:11];
                    end
                end
            end
            if (w_done) wv <= 1'b0;
            if (in_valid && in_ready) begin
                // 비어 있지 않은 bitmap 만 GRP job 을 낸다.
                wv   <= 1'b1;
                wd   <= in_word;
                p_lo <= in_word[31] && (in_word[7:0]  != 8'd0);
                p_hi <= in_word[31] && (in_word[15:8] != 8'd0);
            end
        end
    end
endmodule
