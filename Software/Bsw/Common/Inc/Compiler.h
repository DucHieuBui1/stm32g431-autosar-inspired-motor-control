#ifndef COMPILER_H
#define COMPILER_H

/* Minimal compiler abstraction required by the current miniAUTOSAR baseline. */
#define AUTOMATIC
#define TYPEDEF

#define STATIC                          static
#define INLINE                          inline
#define LOCAL_INLINE                    static inline

#define FUNC(rettype, memclass)         rettype
#define VAR(vartype, memclass)          vartype
#define CONST(consttype, memclass)      const consttype

#define P2VAR(ptrtype, memclass, ptrclass)       ptrtype *
#define P2CONST(ptrtype, memclass, ptrclass)     const ptrtype *
#define CONSTP2VAR(ptrtype, memclass, ptrclass)  ptrtype * const
#define CONSTP2CONST(ptrtype, memclass, ptrclass) const ptrtype * const
#define P2FUNC(rettype, ptrclass, fctname)       rettype (*fctname)

#endif /* COMPILER_H */
