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
        else
            return 0;
    }

    void clear() {
        _end = _begin;
    }

    void push_back(const T& v) {
        if( _begin == NULL ) {
            _begin = new T[1];
            _reserved = _begin + 1;

            _end = _begin;
            *_end = v;
            _end ++;

            return;
        }

        if( _end >= _reserved ) {
            size_t n = _reserved - _begin;
            reserve(2*n);

            *_end = v;
            _end ++;

            return;
        } else {
            *_end = v;
            _end ++;

            return;
        }
    }

    void resize(size_t ns, const T& dv) {
        if( ns > _reserved - _begin ) reserve(ns);

        size_t os = _end - _begin;
        if( ns > os ) {
            for(int i=os; i<ns; i++) _begin[i] = dv;
        }

        _end = _begin + ns;
    }

    void reserve(size_t ns) {
        size_t os = _reserved - _begin;
        T *new_arr, *old_arr;

        if( ns > os ) {
            new_arr = new T[ns];
            old_arr = _begin;

            for(int i=0; i<_end-_begin; i++) new_arr[i] = _begin[i];

            _end = new_arr + (_end - _begin);
            _reserved = new_arr + ns;
            _begin = new_arr;

            delete [] old_arr;
        } else {
            return;
        }
    }

    T& operator[](size_t idx) {
        if( _begin == NULL ) { push_back(T()); return _begin[0]; }

        if( idx < _end - _begin ) { return _begin[idx]; }
    }


protected:
    T*      _begin;
    T*      _end;
    T*      _reserved;
};

} // end of namespace summercamp

#endif // end of _SUMMERCAMP_VECTOR_H
