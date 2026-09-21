#ifndef TRACE_ENTRY_LOG_H_
#define TRACE_ENTRY_LOG_H_

// 線を掴んでから高速区間へ渡すまで（接続）の診断。成功時は走りながら記録だけして、
// 安全に止まった後でまとめて出す。制御の切れ目を作らないため、その場ではsyslogしない。
// 文字列はリテラルだけを入れること（コピーせずポインタだけ持つ）。
struct TraceEntryRecord {
    const char* label = "";
    const char* result = "";
    int mm = 0;
    int ms = 0;
    int heading10 = 0;
    int firstReflection = -1;
    int lastReflection = -1;
    int stable = 0;
    int firstP100 = 0;  // 接続の初回のP/I/D（×100）。再開直後のD項が消えたかの確認用
    int firstI100 = 0;
    int firstD100 = 0;
    int gapToFastUs = -1;  // 最後の低速制御から、最初の高速制御までの時間[us]
};

// 上限を決め打ちにした追記専用のバッファ。溢れた分は捨てて件数だけ数える。
template <int Capacity>
class TraceEntryLogBuffer {
public:
    void clear() {
        count_ = 0;
        dropped_ = 0;
    }

    // 追加した位置を返す。溢れていれば-1
    int add(const TraceEntryRecord& record) {
        if(count_ >= Capacity) {
            dropped_++;
            return -1;
        }
        records_[count_] = record;
        return count_++;
    }

    int count() const { return count_; }
    int dropped() const { return dropped_; }
    static constexpr int capacity() { return Capacity; }
    const TraceEntryRecord& at(int index) const { return records_[index]; }
    TraceEntryRecord& at(int index) { return records_[index]; }

private:
    TraceEntryRecord records_[Capacity]{};
    int count_ = 0;
    int dropped_ = 0;
};

#endif  // !TRACE_ENTRY_LOG_H_
