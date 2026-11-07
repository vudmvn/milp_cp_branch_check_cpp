#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Script chạy thực nghiệm tự động cho MILP-CP Solver
# ============================================================

EXE="./milp_cp_solver"
DATA_DIR="./data"

# Tham số thực nghiệm mặc định
TARGET_N=${1:-30}             # Số khách hàng (30, 50, 100...)
MASTER_TIME=${2:-300}         # Giới hạn thời gian Master (giây)
CP_TIME=${3:-5}               # Giới hạn thời gian CP subproblem (giây)
MODE=${4:-"cut"}              # Chế độ: "cut" (lazy subtour) hoặc "mtz"
N_DRONES=${5:-4}              # Số drone override
SWAP_MINUTES=${6:-3}          # Thời gian đổi pin override (phút)

OUTPUT_DIR="./experiments"
LOG_DIR="${OUTPUT_DIR}/logs_${TARGET_N}_${MODE}"
RESULT_CSV="${OUTPUT_DIR}/results_${TARGET_N}_${MODE}.csv"

mkdir -p "$OUTPUT_DIR"
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "BẮT ĐẦU CHẠY THỰC NGHIỆM"
echo "  Số khách hàng mục tiêu : $TARGET_N"
echo "  Chế độ Master          : $MODE"
echo "  Time limit Master      : ${MASTER_TIME}s"
echo "  Time limit CP          : ${CP_TIME}s"
echo "  Số Drone               : $N_DRONES"
echo "  Thời gian đổi pin      : ${SWAP_MINUTES} phút"
echo "  File kết quả CSV       : $RESULT_CSV"
echo "  Thư mục logs           : $LOG_DIR"
echo "============================================================"

# Kiểm tra file thực thi
if [[ ! -x "$EXE" ]]; then
    echo "[LỖI] Không tìm thấy file thực thi: $EXE"
    echo "Hãy biên dịch trước:"
    echo "  mkdir -p build && cd build && cmake .. && cmake --build . -j && cp milp_cp_solver .. && cd .."
    exit 1
fi

count=0
success=0

for file in "$DATA_DIR"/${TARGET_N}-*.txt; do
    [[ -f "$file" ]] || continue

    base=$(basename "$file" .txt)
    log_file="$LOG_DIR/${base}.log"

    echo "[RUN] $base ..."

    "$EXE" \
        "$file" \
        "$N_DRONES" \
        "$SWAP_MINUTES" \
        "$MASTER_TIME" \
        "$CP_TIME" \
        "$MODE" \
        "$RESULT_CSV" > "$log_file" 2>&1 || {
            echo "  [CẢNH BÁO] Chạy thất bại trên instance: $base"
            continue
        }

    count=$((count + 1))
    success=$((success + 1))
    echo "  -> Hoàn thành (Log: $log_file)"
done

echo "============================================================"
echo "HOÀN TẤT THỰC NGHIỆM!"
echo "  Đã chạy thành công: $success/$count instances."
echo "  Kết quả tổng hợp tại: $RESULT_CSV"
echo "============================================================"
