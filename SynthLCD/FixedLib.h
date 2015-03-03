/*
 * FixedLib.h
 *
 * Created: 12/4/2014 11:16:24 PM
 *  Author: Sam
 */ 


#ifndef FIXEDLIB_H_
#define FIXEDLIB_H_

typedef unsigned char byte;

inline void toFixed(int a, unsigned long &b);

inline void toFixed(double a, unsigned long &b);

inline void toFixed(unsigned short &a, unsigned long &b);

inline void toFixed(int a, unsigned short &b);

inline void toFixed(double a, unsigned short &b);

inline void toFixed(unsigned long &a, unsigned short &b);

inline void toFixed(int a, volatile unsigned long &b);

inline void toFixed(double a, volatile unsigned long &b);

inline void toFixed(unsigned short &a, volatile unsigned long &b);

inline void toFixed(int a, volatile unsigned short &b);

inline void toFixed(double a, volatile unsigned short &b);

inline void toFixed(unsigned long &a, volatile unsigned short &b);

inline unsigned long fixedAdd(unsigned long &lhs, int rhs);

inline unsigned long fixedAdd(int lhs, unsigned long &rhs);

inline unsigned long fixedAdd(unsigned long &lhs, double rhs);

inline unsigned long fixedAdd(double lhs, unsigned long rhs);

inline unsigned long fixedSub(unsigned long &lhs, int rhs);

inline unsigned long fixedSub(int lhs, unsigned long& rhs);

inline unsigned long fixedSub(unsigned long &lhs, double rhs);

inline unsigned long fixedSub(double lhs, unsigned long &rhs);

inline unsigned long fixedMultiply(unsigned long &lhs, unsigned long &rhs);

inline unsigned long fixedMultiply(unsigned long &lhs, int rhs);

inline unsigned long fixedMultiply(int lhs, unsigned long &rhs);

inline unsigned long fixedMultiply(unsigned long &lhs, double rhs);

inline unsigned long fixedMultiply(double lhs, unsigned long &rhs);

inline unsigned long fixedDivide(unsigned long &lhs, unsigned long &rhs);

inline unsigned long fixedDivide(unsigned long &lhs, int rhs);

inline unsigned long fixedDivide(unsigned long lhs, int rhs);

inline unsigned long fixedDivide(int lhs, unsigned long &rhs);

inline unsigned long fixedDivide(unsigned long &lhs, double rhs);

inline unsigned long fixedDivide(double lhs, unsigned long &rhs);

inline unsigned short fixedAdd(unsigned short &lhs, int rhs);

inline unsigned short fixedAdd(int lhs, unsigned short &rhs);

inline unsigned short fixedAdd(unsigned short &lhs, double rhs);

inline unsigned short fixedAdd(double lhs, unsigned short rhs);

inline unsigned short fixedSub(unsigned short &lhs, int rhs);

inline unsigned short fixedSub(int lhs, unsigned short& rhs);

inline unsigned short fixedSub(unsigned short &lhs, double rhs);

inline unsigned short fixedSub(double lhs, unsigned short &rhs);

inline unsigned short fixedMultiply(unsigned short &lhs, unsigned short &rhs);

inline unsigned short fixedMultiply(unsigned short &lhs, int rhs);

inline unsigned short fixedMultiply(int lhs, unsigned short &rhs);

inline unsigned short fixedMultiply(unsigned short &lhs, double rhs);

inline unsigned short fixedMultiply(double lhs, unsigned short &rhs);

inline unsigned short fixedDivide(unsigned short &lhs, unsigned short &rhs);

inline unsigned short fixedDivide(unsigned short &lhs, int rhs);

inline unsigned short fixedDivide(int lhs, unsigned short &rhs);

inline unsigned short fixedDivide(unsigned short &lhs, double rhs);

inline unsigned short fixedDivide(double lhs, unsigned short &rhs);

inline void fixedAdd(unsigned long &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedAdd(unsigned long &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedAdd(unsigned short &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedAdd(unsigned short &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedSub(unsigned long &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedSub(unsigned long &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedSub(unsigned short &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedSub(unsigned short &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedMultiply(unsigned long &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedMultiply(unsigned long &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedMultiply(unsigned short &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedMultiply(unsigned short &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedDivide(unsigned long &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedDivide(unsigned long &res, unsigned short &lhs, unsigned long &rhs);

inline void fixedDivide(unsigned short &res, unsigned long &lhs, unsigned short &rhs);

inline void fixedDivide(unsigned short &res, unsigned short &lhs, unsigned long &rhs);

inline byte fixedToByte(unsigned short &a);

inline byte fixedToByte(unsigned long &a);

#endif /* FIXEDLIB_H_ */