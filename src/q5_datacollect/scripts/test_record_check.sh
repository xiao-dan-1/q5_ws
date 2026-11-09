#!/bin/bash
# record 录后自检的回归测试。
# 自检的判据**只来自 metadata.yaml**，所以这里只喂手搓的 metadata.yaml：
# 不需要真 bag、不需要机器人、不需要发布者（每例都显式 --state-topic，跳过 auto 探测）。
#
# 用法：bash scripts/test_record_check.sh [record 脚本路径]（默认同目录的 record）
set -u
REC="${1:-$(dirname "$0")/record}"
T=$(mktemp -d /tmp/q5_rec_check_XXXXXX)
trap 'rm -rf "$T"' EXIT
fail=0

# mkbag <ep目录名> <话题 条数>...   —— 时长取全局 $DUR（ns）
# 结构照抄真 bag 的 metadata.yaml（含末尾 files: 块 —— 那里也有个 6 空格缩进的 message_count，
# 不照抄就测不出"幽灵话题"那个 bug）
mkbag() {
  d="$T/coll_chk/$1"; mkdir -p "$d"; shift
  sum=0; for c in $(printf '%s\n' "$@" | awk 'NR%2==0'); do sum=$((sum + c)); done
  {
    echo "rosbag2_bagfile_information:"
    echo "  version: 5"
    echo "  storage_identifier: sqlite3"
    echo "  duration:"
    echo "    nanoseconds: $DUR"
    echo "  starting_time:"
    echo "    nanoseconds_since_epoch: 1"
    echo "  message_count: $sum"
    echo "  topics_with_message_count:"
    if [ $# -ge 2 ]; then
      while [ $# -ge 2 ]; do
        echo "    - topic_metadata:"
        echo "        name: $1"
        echo "        type: a/b"
        echo "        serialization_format: cdr"
        echo "        offered_qos_profiles: \"-\""
        echo "      message_count: $2"
        shift 2
      done
    else
      echo "    []"
    fi
    echo "  compression_format: \"\""
    echo "  relative_file_paths:"
    echo "    - ep_0.db3"
    echo "  files:"
    echo "    - path: ep_0.db3"
    echo "      starting_time:"
    echo "        nanoseconds_since_epoch: 1"
    echo "      duration:"
    echo "        nanoseconds: $DUR"
    echo "      message_count: $sum"
  } > "$d/metadata.yaml"
}

CAM=/camera/camera/color/image_raw/compressed
ARM=/wr1_controller/commands
HAND=/hand_controller/commands
DUR=10000000000          # 10 秒
FULL=" $CAM 300 $ARM 1000 $HAND 500 /joint_states 5000 "   # 相机 29.9Hz 臂 99.9 手 49.9 state 499.9

run() {  # run <期望rc> <说明> [record 参数...]
  want=$1; desc=$2; shift 2
  out=$(bash "$REC" chk --root "$T" --check-only --state-topic /joint_states "$@" 2>&1); rc=$?
  if [ "$rc" = "$want" ]; then
    echo "✓ rc=$rc  $desc"
  else
    echo "✗ 期望 rc=$want，实得 rc=$rc —— $desc"; printf '%s\n' "$out" | sed 's/^/     | /'; fail=1
  fi
}

mkbag ep001 $FULL;                                        run 0 "正常集（四条都达标）" --ep 1
mkbag ep002 $CAM 300 $ARM 1000 /joint_states 5000;        run 3 "缺 hand 命令话题" --ep 2
mkbag ep003 $CAM 300 $ARM 1000 $HAND 0 /joint_states 5000; run 3 "hand 话题在但 0 条" --ep 3
mkbag ep004 $CAM 100 $ARM 1000 $HAND 500 /joint_states 5000; run 3 "相机只有 9.9Hz" --ep 4
mkbag ep005 $CAM 271 $ARM 1000 $HAND 500 /joint_states 5000; run 0 "相机 27.0Hz 压线过" --ep 5
mkbag ep006 $CAM 270 $ARM 1000 $HAND 500 /joint_states 5000; run 3 "相机 26.9Hz 差一点" --ep 6
mkdir -p "$T/coll_chk/ep007";                             run 3 "目录在但没 metadata.yaml（没封包）" --ep 7
run 2 "复查一个不存在的集（参数错）" --ep 99

# 跨机那套：relay 只有 ~56Hz 是设计，不能按 500 判
mkbag ep008 $CAM 300 $ARM 1000 $HAND 500 /joint_states_relay 560
run 0 "relay 55.9Hz 达标" --ep 8 --state-topic /joint_states_relay
mkbag ep009 $CAM 300 $ARM 1000 $HAND 500 /joint_states_relay 400
run 3 "relay 39.9Hz 不达标" --ep 9 --state-topic /joint_states_relay

# --sec N：10s 的 bag
mkbag ep010 $FULL
run 3 "--sec 20 但只录到 10s" --ep 10 --sec 20
run 0 "--sec 10 正好够" --ep 10 --sec 10

# 一条话题都没订上（类型解析不出来时 rosbag2 只 WARN 就不订阅）——真 bag 的 metadata 就长这样
mkbag ep011;                                              run 3 "一条话题都没订上（全缺）" --ep 11
o=$(bash "$REC" chk --root "$T" --check-only --ep 11 --state-topic /joint_states 2>&1)
case "$o" in
  *"多了"*) echo "✗ 空 bag 报了假的「多了」—— files: 块那个 message_count 被当成话题了"; fail=1;;
  *) echo "✓ 空 bag 没冒出幽灵话题";;
esac

# ---- 跨机复查：**别处录**的 bag（state 是 relay）+ 本机 auto ----
# 机器人上有 ~/teleop_client → auto 推 /joint_states；bag 里却是 /joint_states_relay。
# 以前照本机推的去 bag 里找 → 找不到 → 误报"缺话题"把好集判死。
# 现在 state 以 **bag 里真有的那条** 为准（次序同 bag2lerobot 的 state_topic=auto）。
mkbag ep012 $CAM 300 $ARM 1000 $HAND 500 /joint_states_relay 560
o=$(bash "$REC" chk --root "$T" --check-only --ep 12 2>&1); rc=$?
if [ "$rc" = 0 ]; then echo "✓ rc=0  跨机复查 relay bag（auto 推 /joint_states、bag 里是 relay）不误杀"
else echo "✗ 期望 rc=0，实得 rc=$rc —— 跨机复查 relay bag 被误判"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi
case "$o" in
  *"bag 里的 state 是 /joint_states_relay"*) echo "✓ 打出了「state 以 bag 为准」的说明";;
  *) echo "✗ 没打出 state 以 bag 为准的说明"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1;;
esac

# 反向：显式点名 relay 去查一个只有 /joint_states 的 bag —— 显式**不覆盖**，缺了照样红
mkbag ep013 $FULL
run 3 "显式 --state-topic relay 查只有 /joint_states 的 bag（点名就按点名的查）" \
      --ep 13 --state-topic /joint_states_relay

# ---- 参数加固：未知选项不能再落成任务名 / 缺值 / --sec 非正整数 ----
o=$(bash "$REC" chk --root "$T" --check-only --ep 1 --rrot x 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  未知选项 --rrot 当场报错（以前会去建一个叫 --rrot 的数据集目录）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— 未知选项没被拦"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi

o=$(bash "$REC" chk --root "$T" --check-only --ep 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  --ep 缺值当场报错（以前 root 会静默变成空串）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— --ep 缺值没被拦"; fail=1; fi

o=$(bash "$REC" chk --root "$T" --check-only --ep 1 --sec abc 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  --sec abc 当场报错（以前错从 timeout 底层冒出来）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— --sec abc 没被拦"; fail=1; fi

# ---- 参数加固 ②：值不能是下一个选项 / 前导零 / --ep 非数字 ----
# `record t --root --sec 10` 以前把 --sec 吃成 root、把 10 吃成任务名 —— 路径和任务名双双改掉
o=$(bash "$REC" chk --root "$T" --check-only --ep 1 --root --sec 10 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  --root 后面跟着另一个选项（--sec）当场报错，不会把 10 吃成任务名"
else echo "✗ 期望 rc=2，实得 rc=$rc —— --root 把下一个选项吃成了值"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi

run 0 "--sec 08 当 8 秒（前导零按十进制，不再被 0* 误拒）" --ep 10 --sec 08
run 0 "--ep 08 当第 8 集（前导零不误拒）" --ep 08 --state-topic /joint_states_relay
run 2 "--ep abc 当场报错（以前错从 bash 的 10# 算术里冒出来）" --ep abc
run 2 "--sec 0 不是正整数" --ep 1 --sec 0

# ---- 位数上限：bash 的 10# 溢出是静默截断，不能让垃圾数字变成路径 ----
run 2 "--ep 20 位数字当场报错（以前 10# 会静默截断成垃圾集号）" --ep 99999999999999999999
run 2 "--sec 20 位数字当场报错" --ep 1 --sec 99999999999999999999
run 0 "--sec 09 当 9 秒（9 位以内、前导零按十进制）" --ep 10 --sec 09

# 自动集号：目录里躺着位数离谱的名字，要被当成"不像集号"跳过，而不是截断后接着编
mkdir -p "$T/coll_auto/ep99999999999999999999" "$T/coll_auto/ep001"
cp "$T/coll_chk/ep001/metadata.yaml" "$T/coll_auto/ep001/"
o=$(bash "$REC" auto --root "$T" --check-only 2>&1); rc=$?
case "$o" in
  *跳过不像集号的目录*99999999999999999999*)
    echo "✓ 超长数字目录被当成不像集号跳过（没参与编集号）";;
  *) echo "✗ 超长数字目录没被跳过"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1;;
esac
case "$o" in
  *coll_auto/ep002*) echo "✓ rc=$rc  自动集号算成 002（不是截断出来的垃圾数字）";;
  *) echo "✗ 自动集号没算成 002"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1;;
esac

# ---- 重复任务名 / 危险任务名：以前静默用最后一个，路径跟着变 ----
o=$(bash "$REC" a b --root "$T" --check-only 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  两个任务名当场报错（以前静默录进 coll_b/）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— 第二个任务名把第一个顶掉了"; fail=1; fi

o=$(bash "$REC" .. --root "$T" --check-only 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  任务名 .. 当场报错（它会让 out 落回 \$root）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— 任务名 .. 没被拦"; fail=1; fi

# ---- --state-topic auto 要等同于不给：否则 check-only 会拿本机推的话题查别处的 bag ----
o=$(bash "$REC" chk --root "$T" --check-only --ep 12 --state-topic auto 2>&1); rc=$?
if [ "$rc" = 0 ] && printf '%s' "$o" | grep -q "按 bag 里这份查"; then
  echo "✓ rc=0  显式 --state-topic auto 等于不给：跨机复查 relay bag 仍按 bag 认 state"
else echo "✗ 显式 auto 没退化成「按 bag 认」（rc=$rc）"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi

# ---- 任务名含空白：不危险，但会让提示和复制粘贴不可靠 ----
o=$(bash "$REC" "a b" --root "$T" --check-only 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  任务名含空格当场报错"
else echo "✗ 期望 rc=2，实得 rc=$rc —— 任务名带空格没被拦"; fail=1; fi

# ---- 集号上限：自动编号越过 9 位就和扫描范围自相矛盾（会编回被跳过的那个号）----
mkdir -p "$T/coll_max/ep999999999"
o=$(bash "$REC" max --root "$T" --check-only 2>&1); rc=$?
if [ "$rc" = 2 ] && printf '%s' "$o" | grep -q "超 9 位上限"; then
  echo "✓ rc=2  自动集号越过 9 位当场报错（不再编出被扫描忽略的号）"
else echo "✗ 期望 rc=2 且提示超上限，实得 rc=$rc"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi

# ---- convert 的参数加固（和 record 同一套；--force 会 rm -rf，参数错必须在动手前红）----
CONV="${2:-$(dirname "$0")/convert}"
crun() {  # crun <期望rc> <说明> [convert 参数...]
  want=$1; desc=$2; shift 2
  out=$(bash "$CONV" chk "$@" 2>&1); rc=$?
  if [ "$rc" = "$want" ]; then
    echo "✓ rc=$rc  $desc"
  else
    echo "✗ 期望 rc=$want，实得 rc=$rc —— $desc"; printf '%s\n' "$out" | sed 's/^/     | /'; fail=1
  fi
}
crun 2 "未知选项 --rrot 当场报错（convert 以前会把它当任务名）" --rrot x
crun 2 "--root 末尾缺值" --root
crun 2 "--root 后面跟着 --force（不许把 --force 吃成 root）" --root --force
crun 2 "--action cmmand 拼错当场报错（别等到 rm -rf 之后）" --action cmmand

# ---- convert：重复任务名 / 危险任务名 ----
o=$(bash "$CONV" a b 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  convert 两个任务名当场报错（以前静默转 coll_b/）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— convert 的第二个任务名把第一个顶掉了"; fail=1; fi

# 这条是**破坏性**的：`convert .. --force` 的 out 正好是 $root，以前会 rm -rf 掉整个采集根
mkdir -p "$T/root/coll_.."
o=$(bash "$CONV" .. --force --root "$T/root" 2>&1); rc=$?
if [ "$rc" = 2 ] && [ -d "$T/root" ]; then
  echo "✓ rc=2  任务名 .. 当场报错，且采集根目录还在（没有 rm -rf 到 \$root）"
else echo "✗ 期望 rc=2 且 \$root 还在，实得 rc=$rc"; printf '%s\n' "$o" | sed 's/^/     | /'; fail=1; fi

o=$(bash "$CONV" "a b" --root "$T" 2>&1); rc=$?
if [ "$rc" = 2 ]; then echo "✓ rc=2  convert 任务名含空格当场报错（和 record 同一份规则）"
else echo "✗ 期望 rc=2，实得 rc=$rc —— convert 没跟上同一条任务名规则"; fail=1; fi

# ---- --ep 必须 >0（ep000 是自动编号永远编不出来的号）----
run 2 "--ep 0 不是合法集号（自动编号从 001 起）" --ep 0
run 2 "--ep 00 同样拒" --ep 00

# ---- --force 的破坏范围：隔离根 + 哨兵 + **阳性对照** ----
# 两个要点（审查提的，都采纳）：
#  (a) <root>/coll_<task> 必须先存在。convert 在 bags 目录不存在时会提前退出（rc=3），
#      删除分支根本到不了 —— 那种情况下"哨兵还在"什么也证明不了，是空测试。
#  (b) 所以除哨兵外还要一个阳性对照：合法 --force 必须把输出目录里的旧文件删掉。
#      "哨兵在 + 对照没了"才同时说明删除分支确实活着、且只删了它该删的那个目录。
: > "$T/coll_chk/KEEP"                     # 哨兵①：$out 的父目录（bag 目录）里
: > "$T/KEEP_OUTSIDE"                      # 哨兵②：任务目录之外

# A) 输出目录还不存在时给 --force：不该删任何东西（也不该在参数层被拒）
bash "$CONV" chk --root "$T" --force >/dev/null 2>&1; rc=$?
if [ "$rc" != 2 ] && [ -f "$T/coll_chk/KEEP" ] && [ -f "$T/KEEP_OUTSIDE" ] \
   && [ -d "$T/coll_chk/ep001" ]; then
  echo "✓ 输出目录不存在时 --force 什么也没删（rc=$rc 已进转换流程）"
else echo "✗ 输出目录不存在时动了不该动的东西（rc=$rc）"; fail=1; fi

# B) 造一个"已完成"的旧数据集，再逐条跑参数错误
mkdir -p "$T/coll_chk/chk/meta"
: > "$T/coll_chk/chk/meta/info.json"
: > "$T/coll_chk/chk/OLD.txt"              # 阳性对照：合法 --force 后必须消失

survived() {  # 两个哨兵 + 阳性对照是否都还在
  [ -f "$T/coll_chk/KEEP" ] && [ -f "$T/KEEP_OUTSIDE" ] && [ -f "$T/coll_chk/chk/OLD.txt" ]
}
badcase() {   # badcase <期望rc> <说明> [convert 参数...]
  local want=$1 desc=$2; shift 2
  bash "$CONV" "$@" >/dev/null 2>&1; local rc=$?
  if [ "$rc" != "$want" ]; then
    echo "✗ $desc：期望 rc=$want，实得 rc=$rc"; fail=1; return
  fi
  if survived; then echo "✓ rc=$rc  $desc：参数错，哨兵/对照一个没动"
  else echo "✗ $desc：参数错却动了文件系统"; fail=1; fi
}
badcase 2 "缺值（--root 末尾）"          chk --root
badcase 2 "缺值（--root 后跟 --force）"   chk --root --force
badcase 2 "未知选项 --rrot"              chk --root "$T" --rrot --force
badcase 2 "重复任务名"                   chk other --root "$T" --force
badcase 2 "非法 action typo"             chk --root "$T" --action typo --force
badcase 2 "任务名带 /（../task）"         ../task --root "$T" --force

# bags 目录不存在：提前退出（rc=3），压根到不了删除分支 —— 这就是 (a) 那条路
bash "$CONV" chk --root "$T/does_not_exist" --force >/dev/null 2>&1; rc=$?
if [ "$rc" = 3 ] && survived; then
  echo "✓ rc=3  bags 目录不存在：提前退出，删除分支没到（哨兵/对照都在）"
else echo "✗ bags 不存在时行为变了（rc=$rc）"; fail=1; fi

# C) 阳性对照：合法 --force 应该只删 $out 里的旧东西
bash "$CONV" chk --root "$T" --force >/dev/null 2>&1; rc=$?
if [ "$rc" != 2 ] && [ ! -e "$T/coll_chk/chk/OLD.txt" ] \
   && [ -f "$T/coll_chk/KEEP" ] && [ -f "$T/KEEP_OUTSIDE" ] && [ -d "$T/coll_chk/ep001" ]; then
  echo "✓ rc=$rc  合法 --force：对照 OLD.txt 被删（删除分支确实活着），哨兵/旧 bag 都在"
else echo "✗ 合法 --force 删多了或没删干净（rc=$rc）"; fail=1; fi

echo "end: fail=$fail"
exit "$fail"
