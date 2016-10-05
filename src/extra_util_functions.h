#pragma once
#include <vector>
#include <math.h>
#include <memory>
#include <iostream>
#include <map>
#include <string>
//#include "vector_pos_neg_index.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


template <typename T>
inline T& get(std::vector<T>& array, long ind) {
  if (0 <= ind && ind < (long) array.size())
    return array[ind];
  long ind_cut = ind % (long) array.size();
  if (ind_cut < 0)
    ind_cut += array.size();
  return array[ind_cut];
}

template <typename T>
inline const T& get(const std::vector<T>& array, long ind) {
  if (0 <= ind && ind < (long) array.size())
    return array[ind];
  long ind_cut = ind % (long) array.size();
  if (ind_cut < 0)
    ind_cut += array.size();
  return array[ind_cut];
}

/*template <typename T>
inline T& get(VectorPosNegIndex<T>& array, long ind) {
  //if (array.exist_at(ind))
    return array[ind];
}

template <typename T>
inline const T& get(const VectorPosNegIndex<T>& array, long ind) {
  if (0 <= ind && ind < (long) array.size())
    return array[ind];
  long ind_cut = ind % (long) array.size();
  if (ind_cut < 0)
    ind_cut += array.size();
  return array[ind_cut];
}
*/

template <typename T>
inline T scalar_mul(const std::vector<T>& v1,
                    const std::vector<T>& v2,
                    long ind1 = 0, long ind2 = 0) {
  size_t v1size = v1.size();
  size_t v2size = v2.size();
  if (v1size != v2size) {
    //std::cout << "vectors in scalar mull are not the same lenght!!" << std::endl
    //          << "l1 = " << v1.size() << " l2 = " << v2.size() << std::endl;
    v1size = std::min(v1size,v2size);
  }
  T out = 0;
  for (long i = 0; i < (long)v1size; i++)
    out += get(v1,ind1+i)*get(v2,ind2+i);
  return out;
}

/*template <typename T>
inline T scalar_mul(const VectorPosNegIndex<T>& v1,
                    const VectorPosNegIndex<T>& v2,
                    long ind1 = 0, long ind2 = 0) {
  size_t v1size = v1.size();
  size_t v2size = v2.size();
  if (v1size != v2size) {
    v1size = std::min(v1size,v2size);
  }
  T out = 0;
  for (long i = -(long)v1.size_neg(); i < (long)v1.size_pos(); i++) {
    out += v1[ind1+i]*v2[ind2+i];
  }
}*/

template <typename T>
inline std::shared_ptr<std::vector<T>> shift(const std::vector<T>& v, int shift){
  std::shared_ptr<std::vector<T>> out(new std::vector<T>(v.size()));
  for (size_t i = 0; i < v.size(); i++) {
    get(*out, i-shift) = v[i];
  }
  return out;
}

template<typename T>
inline T my_max(std::vector<T> array) {
  T max_local  = array[0];
  for (size_t i = 1; i < array.size(); i++) {
    if (max_local < array[i])
      max_local = array[i];
  }
  return max_local;
}

template<typename T>
inline T my_min(std::vector<T> array) {
  T min_local  = array[0];
  for (size_t i = 1; i < array.size(); i++) {
    if (min_local > array[i])
      min_local = array[i];
  }
  return min_local;
}

class WindowPoisition{
public:
  static int x_position;
  static int y_position;
};

inline GLvoid ReSizeGLScene( GLsizei w, GLsizei h ){
  if( h == 0 ) {
     h = 1;
  }
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  glOrtho(0.0, w, 0.0, h, -1.0, 1.0);
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glFlush();
  return;
}

inline int create_window(long long w, long long h,const char* name) {

  int div = 2;
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(glutGet(GLUT_SCREEN_WIDTH)/div,
                     glutGet(GLUT_SCREEN_HEIGHT)/div);
  glutInitWindowPosition(WindowPoisition::x_position, WindowPoisition::y_position);
  WindowPoisition::y_position += glutGet(GLUT_SCREEN_HEIGHT)/div;
  WindowPoisition::x_position += WindowPoisition::y_position / glutGet(GLUT_SCREEN_HEIGHT) * glutGet(GLUT_SCREEN_WIDTH)/div;
  WindowPoisition::y_position = WindowPoisition::y_position % glutGet(GLUT_SCREEN_HEIGHT);
  int discriptor = glutCreateWindow(name);

  glutReshapeFunc(ReSizeGLScene);
  glClearColor (1.0, 1.0, 1.0, 0.0);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0.0, w, 0.0, h, -1.0, 1.0);
  glClear (GL_COLOR_BUFFER_BIT);
  return discriptor;
}

inline void printRect(int bot, int top, int left, int right, double r, double g, double b) {
  glColor3f(r,g,b);
  glBegin(GL_QUADS);
    glVertex3i(left,bot,0);
    glVertex3i(right,bot,0);
    glVertex3i(right,top,0);
    glVertex3i(left,top,0);
  glEnd();
}

inline void printRect(int bot, int top, int left, int right, double gray) {
  printRect(bot,top,left,right,gray,gray,gray);
}

template<typename T>
inline void print(std::vector<T>& y, int windowID) {
  glutSetWindow(windowID);

  glClearColor (1.0, 1.0, 1.0, 0.0);
  //glMatrixMode(GL_PROJECTION);
  //glLoadIdentity();
  glClear (GL_COLOR_BUFFER_BIT);
  glFlush();

  glColor3f(0.5,0.5,0.5);
  glBegin(GL_LINE_STRIP);
    glVertex3f(1.0,1.0,0.0);
    glVertex3f(y.size(),1.0,0.0);
  glEnd();
  T max_y = my_max(y);
  glBegin(GL_LINE_STRIP);
    glVertex3f(1.0,1.0,0.0);
    glVertex3f(1.0,max_y,0.0);
  glEnd();

  for (size_t x = 0; x < y.size()-1; x++) {
    glBegin(GL_LINE_STRIP);
      glVertex3f(x+1,y[x]/(double)max_y+1,0.0);
      glVertex3f(x+1+1,y[x+1]/(double)max_y+1,0.0);
    glEnd();
    if (y[x] == max_y) {
      glColor3f(1.0,0.0,0.0);
      glBegin(GL_LINE_STRIP);
        glVertex3f(x+1,2.0,0.0);
        glVertex3f(x+1,1.0,0.0);
      glEnd();
      glColor3f(0.5,0.5,0.5);
    }
  }
  glFlush();
}

template<typename T>
inline void print(std::vector<T>& y, std::string name) {
  static std::map<std::string,int> window_store;
  if (window_store.find(name) == window_store.end()) {
    int windID = create_window(y.size()+2,3,name.c_str());
    window_store.insert({name,windID});
  }
  /*if (window_store.find(name)->second == 3) {
    for (auto elem : y) {
      std::cout << elem << " ";
    }
  }*/
  print(y,window_store.find(name)->second);
}

/*template<template <class> class Array, class Type>
inline std::shared_ptr<Array<Array<Type>>> transponde(Array<Array<Type>>& matrix) {
  std::shared_ptr<Array<Array<Type>>> result(new Array<Array<Type>>(matrix[0].size(), Array<Type>(matrix.size(),0)));
  for (size_t x = 0; x < matrix.size(); x++) {
    for (size_t y = 0; y < matrix[0].size(); y++) {
      (*result)[y][x] = matrix[x][y];
    }
  }
  return result;
}

template<template <class> class Array, class Type>
inline std::shared_ptr<Array<Array<Type>>> mul(Array<Array<Type>>& A, Array<Array<Type>>& B) {
  if (A.size() != B[0].size())
    return std::shared_ptr<Array<Array<Type>>>(nullptr);
  for (int row = 0; row < A[0].size(); )
}*/
