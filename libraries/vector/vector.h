#ifndef VECTOR_H
#define VECTOR_H

#include <stdlib.h>
#include <inttypes.h>

// Minimal class to replace std::vector
template<typename Data>
class vector {
   int d_size; // Stores no. of actually stored objects
   int d_capacity; // Stores allocated capacity
   Data *d_data; // Stores data
   public:
     vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
     vector(vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) { d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); }; // Copy constuctor
     ~vector() { free(d_data); }; // Destructor
     vector &operator=(vector const &other) { free(d_data); d_size = other.d_size; d_capacity = other.d_capacity; d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); return *this; }; // Needed for memory management
     void push_back(Data const &x) { if (d_capacity == d_size) resize(); d_data[d_size++] = x; }; // Adds new value. If needed, allocates more space
     int size() const { return d_size; }; // Size getter
     Data const &operator[](int idx) const { return d_data[idx]; }; // Const getter
     Data &operator[](int idx) { return d_data[idx]; }; // Changeable getter
   private:
     void resize() { d_capacity = d_capacity ? d_capacity*2 : 1; Data *newdata = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(newdata, d_data, d_size * sizeof(Data)); free(d_data); d_data = newdata; };// Allocates double the old space
};

#endif
