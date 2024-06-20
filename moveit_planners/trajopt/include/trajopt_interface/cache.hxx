#pragma once
#include <algorithm>

template <class KeyT, class ValueT, unsigned bufsize>
class Cache
{
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KeyT keybuf[bufsize];    // circular buffer
  ValueT valbuf[bufsize];  // circular buffer
  int m_i;
  Cache() : m_i(0)
  {
    memset(keybuf, 666, sizeof(keybuf));
  }
  void put(const KeyT& key, const ValueT& value)
  {
    keybuf[m_i] = key;
    valbuf[m_i] = value;
    ++m_i;
    if (m_i == bufsize)
      m_i = 0;
  }
  ValueT* get(const KeyT& key)
  {
    KeyT* it = std::find(&keybuf[0], &keybuf[0] + bufsize, key);
    return (it == &keybuf[0] + bufsize) ? nullptr : &valbuf[it - &keybuf[0]];
  }
};
