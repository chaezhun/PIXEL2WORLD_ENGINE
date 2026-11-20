// NRV SDK(libDELTA_SDK.so)로 .dvs 를 풀어 이벤트를 그대로 덤프한다. 우리 파서(dvs_read.py)를
// 맞대어 볼 기준을 만드는 도구이고, 결과 비교는 check_sdk.py 가 한다.
//
// SDK 배포본에는 헤더가 없으므로 필요한 선언을 아래에 직접 적는다. 함수 이름과 인자는 .so 의 심볼을,
// 구조체 배치(ClassicEvent 12바이트, ClassicEventList 40바이트)와 열거값(OutputType ClassicEvents = 8,
// PlaybackMode Once = 0, FrameGeneration{모드 0 = 프레임 끝 기준, 값})은 .so 의 역어셈블을 따른다.
// 구조체 크기는 아래 static_assert 가 컴파일할 때 확인한다.
// 경로에 공백이 있어 .so 두 개를 공백 없는 폴더로 복사해 빌드한다.
//
//   B=~/wl_build/sdk_dump; mkdir -p $B
//   cp software/delta_sdk/Linux/x64/{libDELTA_SDK.so,libcyusb.so} software/dvs_develop/sdk_dump.cpp $B/
//   cd $B && g++ -O2 -std=c++17 sdk_dump.cpp -L. -lDELTA_SDK -Wl,-rpath,'$ORIGIN' -o sdk_dump
//   ./sdk_dump <파일>.dvs <출력>.bin
//
// 출력은 SDK 가 내준 프레임마다 [int32 머리 5개(1, 960, 720, 카메라 번호, 프레임 번호), int32 이벤트 수]
// 뒤에 ClassicEvent 를 이어 쓴다. 재생은 프레임 끝 표지 기준으로 한 프레임씩 받는다.
// getClassicEvents 로 프레임을 꺼내 가지 않으면 SDK 내부 버퍼가 차서 재생이 첫 프레임에서 멈춘다.
#include <cstdio>
#include <cstdint>
#include <vector>
#include <atomic>
#include <unistd.h>
namespace delta {
struct InputSource;
enum class Status : int {};
enum class PlaybackMode : int {};
enum class OutputType : int {};
struct FrameGeneration { int32_t mode, value; };
struct ClassicEvent { uint16_t x, y; uint8_t polarity; uint32_t timestamp; };
struct ClassicEventList { int32_t h[5]; ClassicEvent* events; int32_t eventCapacity, eventCount; };
Status openFile(const char*, InputSource**);
Status closeFile(InputSource*);
Status startPlayback(InputSource*);
Status stopPlayback(InputSource*);
Status setPlaybackMode(InputSource*, PlaybackMode);
Status setOutputType(InputSource*, OutputType);
Status setFrameGeneration(InputSource*, FrameGeneration);
FrameGeneration frameEndBased(int);
Status setReadOverCallback(InputSource*, void (*)(void*), void*);
Status setEventCountCallback(InputSource*, void (*)(void*, unsigned long), void*);
Status setFrameCountCallback(InputSource*, void (*)(void*, unsigned long), void*);
Status getClassicEvents(InputSource*, ClassicEventList*, int);
}
static_assert(sizeof(delta::ClassicEvent) == 12 && sizeof(delta::ClassicEventList) == 40);
static std::atomic<unsigned long> ev{0}, fr{0}; static std::atomic<bool> done{false};
int main(int c, char** v) {
  delta::InputSource* h = nullptr;
  int s0 = (int)delta::openFile(v[1], &h);
  int s1 = (int)delta::setPlaybackMode(h, (delta::PlaybackMode)0);
  int s2 = (int)delta::setFrameGeneration(h, delta::frameEndBased(1));
  int s3 = (int)delta::setOutputType(h, (delta::OutputType)8);
  delta::setEventCountCallback(h, [](void*, unsigned long n){ ev = n; }, nullptr);
  delta::setFrameCountCallback(h, [](void*, unsigned long n){ fr = n; }, nullptr);
  delta::setReadOverCallback(h, [](void*){ done = true; }, nullptr);
  int s4 = (int)delta::startPlayback(h);
  fprintf(stderr, "status open=%d mode=%d gen=%d out=%d start=%d\n", s0, s1, s2, s3, s4);
  std::vector<delta::ClassicEvent> buf(1 << 21);
  delta::ClassicEventList L{};
  L.events = buf.data(); L.eventCapacity = (int)buf.size();
  FILE* o = fopen(v[2], "wb");
  unsigned long nf = 0, ne = 0; int idle = 0;
  while (idle < 300) {
    int st = (int)delta::getClassicEvents(h, &L, 0);
    if (st == 0) {
      idle = 0;
      if (nf < 3) fprintf(stderr, "frame %lu hdr %d %d %d %d %d count %d\n", nf, L.h[0], L.h[1], L.h[2], L.h[3], L.h[4], L.eventCount);
      fwrite(L.h, 4, 5, o); fwrite(&L.eventCount, 4, 1, o);
      fwrite(L.events, sizeof(delta::ClassicEvent), L.eventCount, o);
      nf++; ne += L.eventCount;
    } else { if (done) idle++; usleep(1000); }
  }
  fclose(o);
  printf("sdk_callback events=%lu frames=%lu | dumped events=%lu frames=%lu\n", ev.load(), fr.load(), ne, nf);
  delta::stopPlayback(h); delta::closeFile(h);
}
