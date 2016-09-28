#pragma once
#include <vector>

template <typename T, typename IndType = long long>
class VectorPosNegIndex {
  std::vector<T> _pos_part;
  std::vector<T> _neg_part;
public:

  using value_type = T;

  VectorPosNegIndex() {}
  VectorPosNegIndex(const std::vector<T>& pos_part,
                    const std::vector<T>& neg_part) :
                      _pos_part(pos_part), _neg_part(neg_part) {}
  VectorPosNegIndex(size_t size) : _pos_part(size) {}
  VectorPosNegIndex(size_t size, const T init_val) : _pos_part(size,init_val) {}

  T& operator [] (IndType i) { return at(i); }
  T& at(IndType i) {
    if (i < 0) {
      IndType correct_index = -i-1;
      expand_if_not_contains(_neg_part, correct_index);
      return _neg_part[correct_index];
    }
    expand_if_not_contains(_pos_part, i);
    return _pos_part[i];
  }

  const T& operator [] (IndType i) const { return at(i); }
  const T& at(IndType i) const {
    if (i < 0)
      return _neg_part[-i-1];
    return _pos_part[i];
  }

  std::vector<T>& get_pos_part() { return _pos_part; }
  const std::vector<T>& get_pos_part() const { return _pos_part; }
  std::vector<T>& get_neg_part() {return _neg_part; }
  const std::vector<T>& get_neg_part() const { return _neg_part; }

  size_t size() const { return _pos_part.size()+_neg_part.size(); }
  size_t size_pos() const { return _pos_part.size(); }
  size_t size_neg() const { return _neg_part.size(); }

  bool exist_at(IndType i) { return -size_neg() <= i && i < size_pos(); }

  class Iterator {
    IndType _i;
    bool _neg_array;
    VectorPosNegIndex<T,IndType>* _value;
  public:
    Iterator(IndType i = 0, bool neg_array = false,
             VectorPosNegIndex<T,IndType>* value = nullptr) :
               _i(i),_neg_array(neg_array),_value(value) {}
    bool operator == (const Iterator& it2) {
      return _value == it2._value &&
             _neg_array == it2._neg_array &&
             _i == it2._i;
    }
    bool operator != (const Iterator& it2) {
      return !(*this == it2);
    }
    Iterator operator = (const Iterator& it2) {
      this->_i = it2._i; this->_neg_array = it2._neg_array;
      this->_value = it2._value;
      return *this;
    }
    Iterator& operator ++ () {
      if (_i == -1 && _neg_array) {
        _neg_array = false;
      }
      _i++;
      return *this;
    }
    Iterator operator + (IndType ind) {
      if (-1 < _i + ind && _neg_array) {
        _neg_array = false;
      }
      _i += ind;
      return *this;
    }
    T& operator *() const {
      return (*_value)[_i];
    }
  };

  Iterator begin() {
    return {(long long)-_neg_part.size(), true, this};
  }
  Iterator end() {
    return {(long long)_pos_part.size(), false, this};
  }

private:
  static void expand_if_not_contains(std::vector<T>& vec, IndType i) {
    if ((long long)vec.size() <= i) {
      IndType count_to_add = i - vec.size() + 1;
      std::vector<T> new_cells(count_to_add);
      vec.insert(vec.end(), new_cells.begin(), new_cells.end());
    }
    return;
  }
};
