#include "../main/app/tasks/AreaBlueGate.h"
#include <cassert>
#include <initializer_list>

int main() {
    using R=AreaBlueGate::Result;
    // 3色とも探索前の青は何回読めても配置しない。途中の取りこぼしにも依存しない。
    for(int expected : {450,735,1005}) {
        AreaBlueGate g(expected-120,expected+200,5);
        for(int i=0;i<20;++i) assert(g.update(expected-200+i,true,true)==R::OBSERVE);
        assert(g.update(expected-100,false,false)==R::OBSERVE); // 未減速では有効にしない
        for(int i=0;i<10;++i) assert(g.update(expected-100+i,true,true)==R::WAIT_CLEAR);
        for(int i=0;i<3;++i) assert(g.update(expected-70+i,false,true)==R::WAIT_CLEAR);
        assert(g.armed());
        for(int i=0;i<4;++i) assert(g.update(expected-10+i,true,true)==R::SEARCH);
        assert(g.update(expected-6,false,true)==R::SEARCH); // 途切れれば連続数をリセット
        for(int i=0;i<4;++i) assert(g.update(expected+i,true,true)==R::SEARCH);
        assert(g.update(expected+4,true,true)==R::FOUND);
    }
    AreaBlueGate missing(885,1205,5);
    for(int i=0;i<3;++i) missing.update(900+i,false,true);
    for(int i=0;i<4;++i) assert(missing.update(1201+i,true,true)==R::SEARCH);
    // 上限を越えた青は5回目であっても採用しない。
    assert(missing.update(1206,true,true)==R::MISSING);
    AreaBlueGate boundary(885,1205,5);
    assert(boundary.update(1205,false,true)!=R::MISSING);
    assert(boundary.update(1206,false,true)==R::MISSING);
}
