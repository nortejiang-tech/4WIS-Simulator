#include "random.hpp"
#include "ziggurat_constants.h"
namespace fourwis {
Random::Random(uint64_t seed) {
  uint32_t pool[4]{}, h = 0x43b0d7e5U;
  auto hash = [&](uint32_t v) {
    v ^= h;
    h *= 0x931e8875U;
    v *= h;
    v ^= v >> 16;
    return v;
  };
  for (int i = 0; i < 4; i++)
    pool[i] = hash(i == 0 ? uint32_t(seed) : i == 1 ? uint32_t(seed >> 32) : 0);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      if (i != j) {
        uint32_t v = 0xca01f9ddU * pool[j] - 0x4973f715U * hash(pool[i]);
        pool[j] = v ^ (v >> 16);
      }
  h = 0x8b51f9ddU;
  uint32_t words[8];
  for (int i = 0; i < 8; i++) {
    uint32_t v = pool[i % 4] ^ h;
    h *= 0x58f38dedU;
    v *= h;
    words[i] = v ^ (v >> 16);
  }
  uint64_t val[4];
  for (int i = 0; i < 4; i++)
    val[i] = uint64_t(words[2 * i]) | (uint64_t(words[2 * i + 1]) << 32);
  pcg64_srandom_r(&state, PCG_128BIT_CONSTANT(val[0], val[1]),
                  PCG_128BIT_CONSTANT(val[2], val[3]));
}
double Random::normal() {
  for (;;) {
    uint64_t r = next();
    int idx = r & 255;
    r >>= 8;
    int s = r & 1;
    uint64_t ab = (r >> 1) & 0x000fffffffffffffULL;
    double x = ab * wi_double[idx];
    if (s)
      x = -x;
    if (ab < ki_double[idx])
      return x;
    if (idx == 0) {
      for (;;) {
        double xx = -ziggurat_nor_inv_r * std::log1p(-uniform()),
               yy = -std::log1p(-uniform());
        if (yy + yy > xx * xx)
          return (ab >> 8) & 1 ? -(ziggurat_nor_r + xx) : ziggurat_nor_r + xx;
      }
    } else if ((fi_double[idx - 1] - fi_double[idx]) * uniform() +
                   fi_double[idx] <
               std::exp(-.5 * x * x))
      return x;
  }
}
static uint32_t rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
std::array<uint8_t, 32> sha256(const std::string &input) {
  static constexpr uint32_t K[] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
      0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
      0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
      0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
      0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
      0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
      0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
      0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
      0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                  0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<uint8_t> b(input.begin(), input.end());
  uint64_t bits = uint64_t(b.size()) * 8;
  b.push_back(128);
  while (b.size() % 64 != 56)
    b.push_back(0);
  for (int i = 7; i >= 0; i--)
    b.push_back(uint8_t(bits >> (8 * i)));
  for (size_t off = 0; off < b.size(); off += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; i++)
      w[i] = (uint32_t(b[off + 4 * i]) << 24) |
             (uint32_t(b[off + 4 * i + 1]) << 16) |
             (uint32_t(b[off + 4 * i + 2]) << 8) | b[off + 4 * i + 3];
    for (int i = 16; i < 64; i++) {
      auto a = w[i - 15], c = w[i - 2];
      w[i] = w[i - 16] + (rotr(a, 7) ^ rotr(a, 18) ^ (a >> 3)) + w[i - 7] +
             (rotr(c, 17) ^ rotr(c, 19) ^ (c >> 10));
    }
    uint32_t a = h[0], bb = h[1], c = h[2], d = h[3], e = h[4], f = h[5],
             g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
      uint32_t t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) +
                    ((e & f) ^ ((~e) & g)) + K[i] + w[i],
               t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) +
                    ((a & bb) ^ (a & c) ^ (bb & c));
      hh = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = bb;
      bb = a;
      a = t1 + t2;
    }
    h[0] += a;
    h[1] += bb;
    h[2] += c;
    h[3] += d;
    h[4] += e;
    h[5] += f;
    h[6] += g;
    h[7] += hh;
  }
  std::array<uint8_t, 32> out{};
  for (int i = 0; i < 8; i++)
    for (int j = 0; j < 4; j++)
      out[4 * i + j] = uint8_t(h[i] >> (24 - 8 * j));
  return out;
}
} // namespace fourwis
