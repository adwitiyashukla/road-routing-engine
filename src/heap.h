#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rr {

template <typename Key>
class MinHeap {
 public:
  struct Item {
    Key key;
    std::uint32_t id;
  };

  void Resize(std::size_t id_bound) { position_.resize(id_bound); }
  bool Empty() const { return items_.empty(); }
  std::size_t Size() const { return items_.size(); }
  Key MinKey() const { return items_[0].key; }
  std::uint32_t MinId() const { return items_[0].id; }
  void Clear() { items_.clear(); }

  void Push(std::uint32_t id, Key key) {
    items_.push_back({key, id});
    SiftUp(items_.size() - 1);
  }

  void Decrease(std::uint32_t id, Key key) {
    std::size_t i = position_[id];
    items_[i].key = key;
    SiftUp(i);
  }

  std::uint32_t Pop() {
    std::uint32_t id = items_[0].id;
    Item last = items_.back();
    items_.pop_back();
    if (!items_.empty()) SiftDown(0, last);
    return id;
  }

  std::vector<Item>& items() { return items_; }

  void Reindex() {
    for (std::size_t i = 0; i < items_.size(); ++i) {
      position_[items_[i].id] = static_cast<std::uint32_t>(i);
    }
  }

 private:
  void SiftUp(std::size_t i) {
    Item item = items_[i];
    while (i > 0) {
      std::size_t parent = (i - 1) / 4;
      if (!(item.key < items_[parent].key)) break;
      items_[i] = items_[parent];
      position_[items_[i].id] = static_cast<std::uint32_t>(i);
      i = parent;
    }
    items_[i] = item;
    position_[item.id] = static_cast<std::uint32_t>(i);
  }

  void SiftDown(std::size_t i, Item item) {
    const std::size_t n = items_.size();
    while (true) {
      std::size_t child = 4 * i + 1;
      if (child >= n) break;
      std::size_t end = std::min(child + 4, n);
      std::size_t best = child;
      for (std::size_t c = child + 1; c < end; ++c) {
        if (items_[c].key < items_[best].key) best = c;
      }
      if (!(items_[best].key < item.key)) break;
      items_[i] = items_[best];
      position_[items_[i].id] = static_cast<std::uint32_t>(i);
      i = best;
    }
    items_[i] = item;
    position_[item.id] = static_cast<std::uint32_t>(i);
  }

  std::vector<Item> items_;
  std::vector<std::uint32_t> position_;
};

}
