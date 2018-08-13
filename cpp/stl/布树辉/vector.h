#ifndef _SUMMERCAMP_VECTOR_H
#define _SUMMERCAMP_VECTOR_H

#include <stdint.h>

namespace summercamp {

template <typename T>
class vector
{
public:
    vector() : _begin(NULL), _end(NULL), _reserved(NULL) {}
    virtual ~vector() { if( _begin ) delete [] _begin; }

    size_t size() { return _end - _begin; }
    size_t capacity() { return _reserved - _begin; }

    int empty() {
        if( _begin == NULL || _end == _begin )
            return 1;
        else return 0;
    }

    push_back(T& v) {
        if( _begin == NULL ) {
            _begin = new T[1];
            _end = _begin + 1;
            _reserved = _begin + 1;

            _cur = _begin;
            *_cur = v;

            _cur ++;
            if( _cur > _end ) _end = _cur;

            return;
        }

        if( _cur >= _reserved ) {
            size_t n = _reserved - _begin;
            T* new_arr = new T[n*2], *old_arr = _begin;

            for(int i=0; i<n; i++) new_arr[i] = _begin[i];
            _cur = new_arr + (_cur - _begin);
            *_cur = v;
            _cur ++;

            _begin = new_arr;
            _end = _cur;
            _reserved = _begin + 2*n;

            delete [] old_arr;

            return;
        } else {
            *_cur = v;
            _cur ++;
            return;
        }
    }

    T& operator[](size_t idx) {
        if( _begin == NULL ) { push_back(T()); return _begin[0]; }

        if( idx < _end - _begin ) { return _begin[idx]; }
    }


protected:
    T*      _cur;
    T*      _begin;
    T*      _end;
    T*      _reserved;
};

} // end of namespace summercamp

#endif // end of _SUMMERCAMP_VECTOR_H
