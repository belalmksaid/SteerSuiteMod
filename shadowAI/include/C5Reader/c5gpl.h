//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef C5ENGINE
#define C5ENGINE

/*************************************************************************/
/*																		 */
/*  Copyright 2010 Rulequest Research Pty Ltd.							 */
/*																		 */
/*  This file is part of C5.0 GPL Edition, a single-threaded version	 */
/*  of C5.0 release 2.07.												 */
/*																		 */
/*  C5.0 GPL Edition is free software: you can redistribute it and/or	 */
/*  modify it under the terms of the GNU General Public License as	     */
/*  published by the Free Software Foundation, either version 3 of the	 */
/*  License, or (at your option) any later version.						 */
/*																		 */
/*  C5.0 GPL Edition is distributed in the hope that it will be useful,	 */
/*  but WITHOUT ANY WARRANTY; without even the implied warranty of	     */
/*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU	 */
/*  General Public License for more details.				             */
/*									                                     */
/*  You should have received a copy of the GNU General Public License	 */
/*  (gpl.txt) along with C5.0 GPL Edition.  If not, see 				 */
/*																		 */
/*      <http://www.gnu.org/licenses/>.									 */
/*																		 */
/*************************************************************************/

/*************************************************************************/
/*																		 */
/*		Modified by Cory D. Boatright									 */
/*		University of Pennsylvania										 */
/*		Starting May 5, 2012											 */
/*																		 */
/*		Changes: Ported to Windows OS, created static libs and DLLs      */
/*																		 */
/*************************************************************************/

/*************************************************************************/
/*																		 */
/*		                Definitions used in C5.0						 */
/*                      ------------------------						 */
/*																		 */
/*************************************************************************/

#define CINTERFACE	//if this isn't here, WIN32 libraries throw a fit
#define	 RELEASE	"2.07 GPL Edition"

				/*  Uncomment following line to enable
				    sample estimates for large datasets.
				    This can lead to some variablility,
				    especially when used with SMP  */
//#define	SAMPLE_ESTIMATES

#include <cstdio>
#include <math.h>
#include <string.h>
#include <cstdlib>
#include <time.h>
#include <ctype.h>
#include <limits.h>
#include <float.h>
#include <iostream>

/*************************************************************************/
/*									 */
/*		Definitions dependent on cc options			 */
/*									 */
/*************************************************************************/

#define Goodbye(x)		{std::cerr << "\nBailed!\n"; std::cout.flush(); std::cerr.flush(); Cleanup();}

#include <assert.h>
#ifdef _DEBUG
#define	Verbosity(d,s)		if(VERBOSITY >= d) {s;}
#else
#define Verbosity(d,s)
#endif

#if defined(COMPILE_C5DLL)
#define C5_API __declspec(dllexport)
#elif defined(USE_C5DLL)
#define C5_API __declspec(dllimport)
#else
#define C5_API
#endif



/*  Alternative random number generator  */

#define	AltSeed(x)		srand48(x)

#define Free(x)			{free(x); x=0;}


/*************************************************************************/
/*									 */
/*		Constants, macros etc.					 */
/*									 */
/*************************************************************************/

#define	 THEORYFRAC	0.23	/* discount rate for estimated coding cost */

#define	 Nil	   0		/* null pointer */
#define	 false	   0
#define	 true	   1
#define	 None	   -1
#define	 Epsilon   1E-4
#define	 MinLeaf   0.05		/* minimum weight for non-null leaf */
#define	 Width	   80		/* approx max width of output */

#define  EXCLUDE   1		/* special attribute status: do not use */
#define  SKIP	   2		/* do not use in classifiers */
#define  DISCRETE  4		/* ditto: collect values as data read */
#define  ORDERED   8		/* ditto: ordered discrete values */
#define  DATEVAL   16		/* ditto: YYYY/MM/DD or YYYY-MM-DD */
#define  STIMEVAL  32		/* ditto: HH:MM:SS */
#define	 TSTMPVAL  64		/* date time */

#define	 CMINFO	   1		/* generate confusion matrix */
#define	 USAGEINFO 2		/* print usage information */

				/* unknown and N/A values are represented by
				   unlikely floating-point numbers
				   (octal 01600000000 and 01) */
#define	 UNKNOWN   01600000000	/* 1.5777218104420236e-30 */
#define	 NA	   01		/* 1.4012984643248171e-45 */

#define	 BrDiscr   1
#define	 BrThresh  2
#define	 BrSubset  3

#define  Plural(n)		((n) != 1 ? "s" : "")

#define  AllocZero(N,T)		(T *) Pcalloc(N, sizeof(T))
#define  Alloc(N,T)		AllocZero(N,T) /* for safety */
#define  Realloc(V,N,T)		V = (T *) Prealloc(V, (N)*sizeof(T))

#define	 Max(a,b)               ((a)>(b) ? (a) : (b))
#define	 Min(a,b)               ((a)<(b) ? (a) : (b))

#define	 Log2			0.69314718055994530942
#define	 Log(x)			((x) <= 0 ? 0.0 : log((double)x) / Log2)

#define	 Bit(b)			(1 << (b))
#define	 In(b,s)		((s[(b) >> 3]) & Bit((b) & 07))
#define	 ClearBits(n,s)		memset(s,0,n)
#define	 CopyBits(n,f,t)	memcpy(t,f,n)
#define	 SetBit(b,s)		(s[(b) >> 3] |= Bit((b) & 07))
#define	 ResetBit(b,s)		(s[(b) >> 3] ^= Bit((b) & 07))

#define	 ForEach(v,f,l)		for(v=f ; v<=l ; ++v)

#define	 CountCases(f,l)	(UnitWeights ? (l-(f)+1.0) : SumWeights(f,l))

#define	 StatBit(a,b)		(SpecialStatus[a]&(b))
#define	 Exclude(a)		StatBit(a,EXCLUDE)
#define	 Skip(a)		StatBit(a,EXCLUDE|SKIP)
#define  Discrete(a)		(MaxAttVal[a] || StatBit(a,DISCRETE))
#define  Continuous(a)		(! MaxAttVal[a] && ! StatBit(a,DISCRETE))
#define	 Ordered(a)		StatBit(a,ORDERED)
#define	 DateVal(a)		StatBit(a,DATEVAL)
#define	 TimeVal(a)		StatBit(a,STIMEVAL)
#define	 TStampVal(a)		StatBit(a,TSTMPVAL)

#define  FreeUnlessNil(p)	if((p)!=Nil) Free(p)

#define  CheckClose(f)		if(f) {fclose(f); f=Nil;}

#define	 Int(x)			((int)(x+0.5))

#define  Space(s)	(s == ' ' || s == '\n' || s == '\r' || s == '\t')
#define  SkipComment	while ( ( c = InChar(f) ) != '\n' && c != EOF )

#define	 P1(x)		(rint((x)*10) / 10)

#define	 No(f,l)	((l)-(f)+1)

#define	 EmptyNA(T)	(T->Branch[1]->Cases < 0.01)

#define  Before(n1,n2)  (n1->Tested < n2->Tested ||\
			n1->Tested == n2->Tested && n1->Cut < n2->Cut)

#define	 Swap(a,b)	{DataRec xab;\
			 assert(a >= 0 && a <= MaxCase &&\
			        b >= 0 && b <= MaxCase);\
			 xab = Case[a]; Case[a] = Case[b]; Case[b] = xab;}


#define	 NOFILE		 0
#define	 BADCLASSTHRESH	 1
#define	 LEQCLASSTHRESH	 2
#define	 BADATTNAME	 3
#define	 EOFINATT	 4
#define	 SINGLEATTVAL	 5
#define	 BADATTVAL	 6
#define	 BADNUMBER	 7
#define	 BADCLASS	 8
#define	 BADCOSTCLASS	 9
#define	 BADCOST	10
#define	 NOMEM		11
#define	 TOOMANYVALS	12
#define	 BADDISCRETE	13
#define	 NOTARGET	14
#define	 BADCTARGET	15
#define	 BADDTARGET	16
#define	 LONGNAME	17
#define	 HITEOF		18
#define	 MISSNAME	19
#define	 BADDATE	20
#define	 BADTIME	21
#define	 BADTSTMP	22
#define	 DUPATTNAME	23
#define	 UNKNOWNATT	24
#define	 BADDEF1	25
#define	 BADDEF2	26
#define	 BADDEF3	27
#define	 BADDEF4	28
#define	 SAMEATT	29
#define	 MODELFILE	30
#define	 CWTATTERR	31

#define	 READDATA	 1
#define	 WINNOWATTS	 2
#define	 FORMTREE	 3
#define	 SIMPLIFYTREE	 4
#define	 FORMRULES	 5
#define	 SIFTRULES	 6
#define	 EVALTRAIN	 7
#define	 READTEST	 8
#define	 EVALTEST	 9
#define	 CLEANUP	10
#define	 ALLOCTABLES	11
#define	 RESULTS	12
#define	 READXDATA	13


/*************************************************************************/
/*									 */
/*		Type definitions					 */
/*									 */
/*************************************************************************/


typedef  unsigned char	BranchType, *Set, Byte;
typedef	 char		*String;

typedef  int	CaseNo;		/* data case number */
typedef  float	CaseCount;	/* count of (partial) cases */

typedef  int	ClassNo,	/* class number, 1..MaxClass */
		DiscrValue,	/* discrete attribute value */
		Attribute;	/* attribute number, 1..MaxAtt */

#ifdef USEDOUBLE
typedef	 double	ContValue;	/* continuous attribute value */
#define	 PREC	14		/* precision */
#else
typedef	 float	ContValue;	/* continuous attribute value */
#define	 PREC	 7		/* precision */
#endif


typedef  union	 _def_val
	 {
	    String	_s_val;		/* att val for comparison */
	    ContValue	_n_val;		/* number for arith */
	 }
	 DefVal;

typedef  struct  _def_elt
	 {
	    short	_op_code;	/* type of element */
	    DefVal	_operand;	/* string or numeric value */
	 }
	 DefElt, *Definition;

typedef  struct  _elt_rec
	 {
	    int		Fi,		/* index of first char of element */
			Li;		/* last ditto */
	    char	Type;		/* 'B', 'S', or 'N' */
	 }
	 EltRec;

#define	 DefOp(DE)	DE._op_code
#define	 DefSVal(DE)	DE._operand._s_val
#define	 DefNVal(DE)	DE._operand._n_val

#define	 OP_ATT			 0	/* opcodes */
#define	 OP_NUM			 1
#define	 OP_STR			 2
#define	 OP_MISS		 3
#define	 OP_AND			10
#define	 OP_OR			11
#define	 OP_EQ			20
#define	 OP_NE			21
#define	 OP_GT			22
#define	 OP_GE			23
#define	 OP_LT			24
#define	 OP_LE			25
#define	 OP_SEQ			26
#define	 OP_SNE			27
#define	 OP_PLUS		30
#define	 OP_MINUS		31
#define	 OP_UMINUS		32
#define	 OP_MULT		33
#define	 OP_DIV			34
#define	 OP_MOD			35
#define	 OP_POW			36
#define	 OP_SIN			40
#define	 OP_COS			41
#define	 OP_TAN			42
#define	 OP_LOG			43
#define	 OP_EXP			44
#define	 OP_INT			45
#define	 OP_END			99


typedef  union  _attribute_value
	 {
	    DiscrValue	_discr_val;
	    ContValue	_cont_val;
	 }
	 AttValue, *DataRec;

typedef	 struct _sort_rec
	 {
	    ContValue	V;
	    ClassNo	C;
	    float	W;
	 }
	 SortRec;

#define  CVal(Case,Att)		Case[Att]._cont_val
#define  DVal(Case,Att)		Case[Att]._discr_val
#define  XDVal(Case,Att)	(Case[Att]._discr_val & 077777777)
#define  SVal(Case,Att)		Case[Att]._discr_val
#define  Class(Case)		(*Case)._discr_val
#define  Weight(Case)		(*(Case-1))._cont_val

#define	 Unknown(Case,Att)	(DVal(Case,Att)==UNKNOWN)
#define	 UnknownVal(AV)		(AV._discr_val==UNKNOWN)
#define	 NotApplic(Case,Att)	(Att != ClassAtt && DVal(Case,Att)==NA)
#define	 NotApplicVal(AV)	(AV._discr_val==NA)

#define	 RelCWt(Case)		( Unknown(Case,CWtAtt)||\
				  NotApplic(Case,CWtAtt)||\
			  	  CVal(Case,CWtAtt)<=0 ? 1 :\
				  CVal(Case,CWtAtt)/AvCWt )

typedef  struct _treerec	*Tree;
typedef  struct _treerec
	 {
	    BranchType	NodeType;
	    ClassNo	Leaf;		/* best class at this node */
	    CaseCount	Cases,		/* no of cases at this node */
			*ClassDist,	/* class distribution of cases */
	    		Errors;		/* est or resub errors at this node */
	    Attribute	Tested; 	/* attribute referenced in test */
	    int		Forks,		/* number of branches at this node */
			Leaves;		/* number of non-empty leaves in tree */
	    ContValue	Cut,		/* threshold for continuous attribute */
		  	Lower,		/* lower limit of soft threshold */
		  	Upper,		/* upper limit ditto */
			Mid;		/* midpoint for soft threshold */
	    Set         *Subset;	/* subsets of discrete values  */
	    Tree	*Branch,	/* Branch[x] = subtree for outcome x */
			Parent;		/* node above this one */
	 }
	 TreeRec;


typedef	 struct _environment
	 {
	    CaseNo	Xp, Ep;			/* start and end of scan  */
	    double	Cases,			/* total cases */
			KnownCases,		/* ditto less missing values */
			ApplicCases,		/* cases with numeric values */
			HighCases, LowCases,	/* cases above/below cut */
			NAInfo,			/* info for N/A values */
			FixedSplitInfo,		/* split info for ?, N/A */
			BaseInfo,		/* info before split */
			UnknownRate,		/* proportion of ? values */
			MinSplit,		/* min cases before/after cut */
			**Freq,			/* local Freq[4][class] */
			*ClassFreq,		/* local class frequencies */
			*ValFreq;		/* cases with val i */
	    ClassNo	HighClass, LowClass;	/* class after/before cut */
	    ContValue	HighVal, LowVal;	/* values after/before cut */
	    SortRec	*SRec;			/* for Cachesort() */
	    Set		**Subset,		/* Subset[att][number] */
			*WSubset;		/* working subsets */
	    int		*Subsets,		/* no of subsets for att */
			Blocks,			/* intermediate no of subsets */
			Bytes,			/* size of each subset */
			ReasonableSubsets;
	    double	*SubsetInfo,		/* subset info */
			*SubsetEntr,		/* subset entropy */
			**MergeInfo,		/* info of merged subsets i,j */
			**MergeEntr;		/* entropy ditto */
	 }
	 EnvRec;


typedef  int	RuleNo;			/* rule number */

typedef  struct _condrec
	 {
	    BranchType	NodeType;	/* test type (see tree nodes) */
	    Attribute	Tested;		/* attribute tested */
	    ContValue	Cut;		/* threshold (if relevant) */
	    Set		Subset;		/* subset (if relevant) */
	    int		TestValue,	/* specified outcome of test */
			TestI;		/* rule tree index of this test */
	 }
	 CondRec, *Condition;


typedef  struct _rulerec
	 {
	    RuleNo	RNo;		/* rule number */
	    int		TNo,		/* trial number */
	    		Size;		/* number of conditions */
	    Condition	*Lhs;		/* conditions themselves */
	    ClassNo	Rhs;		/* class given by rule */
	    CaseCount	Cover,		/* number of cases covered by rule */
			Correct;	/* number on which correct */
	    float	Prior;		/* prior probability of RHS */
	    int		Vote;		/* unit = 0.001 */
	 }
	 RuleRec, *CRule;


typedef  struct _ruletreerec *RuleTree;
typedef  struct _ruletreerec
	 {
	    RuleNo	*Fire;		/* rules matched at this node */
	    Condition	CondTest;	/* new test */
	    int		Forks;		/* number of branches */
	    RuleTree	*Branch;	/* subtrees */
	 }
	 RuleTreeRec;


typedef struct _rulesetrec
	 {
	    RuleNo	SNRules;	/* number of rules */
	    CRule	*SRule;		/* rules */
	    ClassNo	SDefault;	/* default class for this ruleset */
	    RuleTree	RT;		/* rule tree (see ruletree.c) */
	 }
	 RuleSetRec, *CRuleSet;

typedef struct _datablockrec	*DataBlock;

typedef	struct _datablockrec
	{
	  DataRec	Head;		/* first address */
	  int		Allocated;	/* number of cases in this block */
	  DataBlock	Prev;		/* previous data block */
	}
	DataBlockRec;

typedef  union  _xstack_elt
         {
            DiscrValue  _discr_val;
            ContValue   _cont_val;
            String      _string_val;
         }
	 XStackElt;

	/*  General stuff  */

//#define UTF8		 	/* uncomment if using UTF-8 */

#ifdef UTF8
#define	 CharWidth(S)		UTF8CharWidth(S)
#else
#define	 CharWidth(S)		(int) strlen(S)
#endif

	/*  Strings with width/format restrictions  */
	/*  (W = width when printed, C = centered)  */

#define	 F_Fold			"Fold"				/* W<8 */
#define	 F_UFold		"----"				/* W same */
#define	 F_Trial		"Trial"				/* W<8 */
#define	 F_UTrial		"-----"				/* W same */
#define	 F_DecisionTree16	"  Decision Tree "		/* W=16C */
#define	 F_SizeErrors		"Size      Errors"		/* W=16 */
#define	 F_DecisionTree23	"     Decision Tree     "	/* W=23C */
#define	 F_SizeErrorsCost	"Size      Errors   Cost"	/* W=23 */
#define	 F_Rules16		"      Rules     "		/* W=16C */
#define	 F_NoErrors		"  No      Errors"		/* W=16 */
#define	 F_Rules23		"         Rules         "	/* W=23C */
#define	 F_NoErrorsCost		"  No      Errors   Cost"	/* W=23 */
#define	 F_Rules		"Rules"				/* W<8 */
#define	 F_URules		"-----"				/* W same */
#define	 F_Errors		"Errors"			/* W<8 */
#define	 F_UErrors		"------"			/* W same */
#define	 F_Cost			"Cost"				/* W<8 */
#define	 F_UCost		"----"				/* W same */
#define	 F_Boost		"boost"				/* W<8 */


	/*  Strings of arbitrary length  */

#define	 T_See5			"See5"
#define	 T_C50			"C5.0"
#define	 TX_Release(n)		"Release " n

#define	 T_OptHeader		"\n    Options:\n"
#define	 T_OptApplication	"\tApplication `%s'\n"
#define	 T_OptBoost		"\tBoosted classifiers\n"
#define	 T_OptProbThresh	"\tProbability thresholds\n"
#define	 T_OptTrials		"\t%d boosting trials\n"
#define	 T_OptSubsets		"\tTests on discrete attribute groups\n"
#define	 T_OptMinCases		"\tTests require 2 branches with >=%g cases\n"
#define	 T_OptCF		"\tPruning confidence level %g%%\n"
#define	 T_OptRules		"\tRule-based classifiers\n"
#define	 T_OptSampling		"\tUse %g%% of data for training\n"
#define	 T_OptSeed		"\tRandom seed %d\n"
#define	 T_OptUtility		"\tRule utility ordering (1/%d's)\n"
#define	 T_OptNoCosts		"\tFocus on errors (ignore costs file)\n"
#define	 T_OptWinnow		"\tWinnow attributes\n"
#define	 T_OptNoGlobal		"\tDo not use global tree pruning\n"
#define	 T_OptXval		"\tCross-validate using %d folds\n"
#define	 T_UnregnizedOpt	"\n    **  Unrecognised option %s %s\n"
#define	 T_SummaryOpts		"    **  Summary of options for c5.0:\n"
#define	 T_ListOpts		"\t-f <filestem>\tapplication filestem\n"\
				"\t-r\t\tuse rule-based classifiers\n"\
				"\t-u <bands>\torder rules by utility in"\
					" bands\n"\
				"\t-w\t\tinvoke attribute winnowing\n"\
				"\t-b\t\tinvoke boosting\n"\
				"\t-t <trials>\tnumber of boosting trials\n"\
				"\t-p\t\tuse soft thresholds\n"\
				"\t-e\t\tfocus on errors (ignore costs file)\n"\
				"\t-s\t\tfind subset tests for discrete atts\n"\
				"\t-g\t\tdo not use global tree pruning\n"\
				"\t-m <cases>\trestrict allowable splits\n"\
				"\t-c <percent>\tconfidence level (CF) for"\
					" pruning\n"\
				"\t-S <percent>\ttraining sample percentage\n"\
				"\t-X <folds>\tcross-validate\n"\
				"\t-I <integer>\trandom seed for sampling"\
					" and cross-validation\n"\
				"\t-h\t\tprint this message\n"
#define	 T_UBWarn		"    **  Warning (-u): rule ordering "\
					"has no effect on boosting\n"
#define	 T_ClassVar		"\nClass specified by attribute `%s'\n"
#define	 TX_ReadData(c,a,f)	"\nRead %d cases (%d attributes) from"\
					" %s.data\n", c, a, f
#define	 TX_ReadTest(c,f)	"Read %d cases from %s.test\n", c, f
#define	 T_ReadCosts		"Read misclassification costs from %s.costs\n"
#define	 T_CWtAtt		"Using relative case weighting\n"
#define	 T_AttributesIn		"\nAttributes included:\n"
#define	 T_AttributesOut	"\nAttributes excluded:\n"
#define	 T_AttributesWinnowed	"\n%d attribute%s winnowed\n"
#define	 T_EstImportance	"Estimated importance of remaining"\
					" attributes:\n\n"
#define	 T_NoWinnow		"\nNo attributes winnowed\n"
#define	 T_EvalTrain		"\n\nEvaluation on training data (%d cases):\n"
#define	 T_Usage		"\n\n\tAttribute usage:\n\n"
#define	 T_EvalTest		"\nEvaluation on test data (%d cases):\n"
#define	 T_Time			"\n\nTime: %.1f secs\n"

#define	 T_IgnoreBadClass	"*** ignoring cases with bad or unknown class\n"

#define	 T_Subtree		"\nSubTree [S%d]\n"
#define	 T_ElementOf		"in"
#define	 T_InRange		"in"
#define	 T_RuleHeader		"\nRule "
#define	 T_RuleLift		", lift %.1f)\n"
#define	 T_IsUnknown		" is unknown\n"

#define	 TX_Reduced1(t)		( (t) > 1 ?\
				   "\n*** boosting reduced to %d trials since"\
					" last classifier is very accurate\n" :\
				   "\n*** boosting reduced to %d trial since"\
					" last classifier is very accurate\n" )
#define	 TX_Reduced2(t)		( (t) > 1 ?\
				   "\n*** boosting reduced to %d trials since"\
					" last classifier is very inaccurate\n" :\
				   "\n*** boosting reduced to %d trial since"\
					" last classifier is very inaccurate\n" )
#define	 T_Abandoned		"\n*** boosting abandoned (too few classifiers)\n"
#define	 T_BoostingUnhelpful	"\n*** warning: boosting may be unhelpful\n"
#define	 T_Composite		"Composite ruleset:"
#define	 T_Tree			"Decision tree:"
#define	 T_Rules		"Rules:"

#define	 T_Default_class	"Default class"
#define	 T_boost		"boost"
#define	 T_composite_ruleset	"composite ruleset"
#define	 T_Rule_utility_summary	"Rule utility summary"
#define	 T_class		"class"
#define	 T_classified_as	"classified as"
#define	 T_Summary		"Summary"
#define	 T_FoldsReduced		"\n*** folds reduced to number of cases\n"
#define	 T_EvalHoldOut		"\nEvaluation on hold-out data (%d cases):\n"
#define	 T_Summary		"Summary"
#define	 T_Fold			"Fold"
#define	 T_Mean			"Mean"
#define	 T_SE			"SE"

#define	 TX_Line(l,f)		"*** line %d of `%s': ", l, f
#define	 E_NOFILE(f,e)		"cannot open file %s%s\n", f, e
#define	 E_ForWrite		" for writing"
#define	 E_BADCLASSTHRESH	"bad class threshold `%s'\n"
#define	 E_LEQCLASSTHRESH	"class threshold `%s' <= previous threshold\n"
#define	 E_BADATTNAME		"`:' or `:=' expected after attribute name"\
					" `%s'\n"
#define	 E_EOFINATT		"unexpected eof while reading attribute `%s'\n"
#define	 E_SINGLEATTVAL(a,v)	"attribute `%s' has only one value `%s'\n",\
					a, v
#define	 E_DUPATTNAME		"multiple attributes with name `%s'\n"
#define	 E_CWTATTERR		"case weight attribute must be continuous\n"
#define	 E_BADATTVAL(v,a)	"bad value of `%s' for attribute `%s'\n", v, a
#define	 E_BADNUMBER(a)		"value of `%s' changed to `?'\n", a
#define	 E_BADCLASS		"bad class value `%s'\n"
#define	 E_BADCOSTCLASS		"bad class `%s'\n"
#define	 E_BADCOST		"bad cost value `%s'\n"
#define	 E_NOMEM		"unable to allocate sufficient memory\n"
#define	 E_TOOMANYVALS(a,n)	"too many values for attribute `%s'"\
					" (max %d)\n", a, n
#define	 E_BADDISCRETE		"bad number of discrete values for attribute"\
					" `%s'\n"
#define	 E_NOTARGET		"target attribute `%s' not found or"\
					" type `ignore'\n"
#define	 E_BADCTARGET		"target attribute `%s' must be"\
					" type `continuous'\n"
#define	 E_BADDTARGET		"target attribute `%s' must be specified by"\
					" a list of discrete values\n"
#define	 E_LONGNAME		"overlength name: check data file formats\n"
#define	 E_HITEOF		"unexpected end of file\n"
#define	 E_MISSNAME		"missing name or value before `%s'\n"
#define	 E_BADTSTMP(d,a)	"bad timestamp `%s' for attribute `%s'\n", d, a
#define	 E_BADDATE(d,a)		"bad date `%s' for attribute `%s'\n", d, a
#define	 E_BADTIME(d,a)		"bad time `%s' for attribute `%s'\n", d, a
#define	 E_UNKNOWNATT		"unknown attribute name `%s'\n"
#define	 E_BADDEF1(a,s,x)	"in definition of attribute `%s':\n"\
					"\tat `%.12s': expect %s\n", a, s, x
#define	 E_BADDEF2(a,s,x)	"in definition of attribute `%s':\n"\
					"\t`%s': %s\n", a, s, x
#define	 E_SAMEATT(a,b)		"[warning] attribute `%s' is identical to"\
					" attribute `%s'\n", a, b
#define	 E_BADDEF3		"cannot define target attribute `%s'\n"
#define	 E_BADDEF4		"[warning] target attribute appears in"\
					" definition of attribute `%s'\n"
#define	 EX_MODELFILE(f)	"file %s incompatible with .names file\n", f
#define	 E_MFATT		"undefined or excluded attribute"
#define	 E_MFATTVAL		"undefined attribute value"
#define	 E_MFCLASS		"undefined class"
#define	 E_MFEOF		"unexpected eof"
#define	 T_ErrorLimit		"\nError limit exceeded\n"
#define	 TX_IllegalValue(v,l,h)	"\t** illegal value %g -- "\
				"should be between %g and %g\n", v, l, h

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

class C5_API C5Engine {

private:

	int VERBOSITY, TRIALS, FOLDS, UTILITY, NCPU;
	int	IValsSize, IValsOffset;
	int MaxAtt, MaxClass, MaxDiscrVal, MaxLabel, LineNo, ErrMsgs, AttExIn, TSBase;
	int Trial, MaxTree, KRInit, Now;
	int *Subsets, *CovBy, *List, *AttValues, *PossibleCuts, *UtilErr, *UtilBand;

	bool SUBSET, BOOST, PROBTHRESH, RULES, XVAL, NOCOSTS, WINNOW, GLOBAL, LOCK;
	bool *SomeMiss, *SomeNA, Winnowed, UnitWeights, CostWeights;

	CaseCount MINITEMS, LEAFRATIO;

	float AttTestBits, *BranchBits, *Gain, *Info, *EstMaxGR, *ClassSum;
	float CF, SAMPLE, SampleFrac;
	float *Vote, *BVoteBlock, **MCost, **NCost, *WeightMul;

	Attribute ClassAtt, LabelAtt, CWtAtt, **AttDefUses;

	double *UtilCost, *LogCaseNo, *LogFact, *ClassFreq, **DFreq, **Bell;
	double GlobalBaseInfo, AvCWt;

	String *ClassName, *AttName, **AttValName, FileStem;

	char *IgnoredVals, *SpecialStatus;
	char Fn[1000];
	
	DiscrValue* MaxAttVal;

	Definition* AttDef;
	
	ContValue *ClassThresh;

	CaseNo MaxCase;

	DataRec *Case, *SaveCase;

	Tree *Raw, *Pruned, WTree;

	CRule *MostSpec;

	ClassNo *TrialPred, Default;

	ContValue* Bar;

	Byte* Tested, **Fires, *CBuffer;

	Set** Subset;

	EnvRec GEnv;

	CRule* Rule;

	RuleNo NRules, RuleSpace;

	CRuleSet* RuleSet;

	FILE *TRf, *Of, *Uf;

	///////////////
	//These variables are used in specific sections of the code.
	//They were removed from being global to support multiple
	//instances of the class, some may need to be made local with
	//proper passing for many-read functionality
	///////////////
	DataRec* Blocked;
	float** Result;
	DataBlock DataMem;
	int DataBlockSize;
	int	KRFp, KRSp;
	char LabelBuffer[1000];
	int	SubTree, SubSpace;
	Tree* SubDef;
	bool LastBranch[Width];
	
	float *DeltaErrs, *Bits, BitsErr, BitsOK;

	int** TotVote;

	ClassNo	*TopClass, *AltClass;

	bool *RuleIn, *Covered;
	Byte* CovByBlock; 
	Byte** CovByPtr;
	RuleNo* LastCovBy;

	Condition* Test;
	int NTest, TestSpace, *TestOccur, *RuleCondOK;
	bool* TestUsed;

	Set* PossibleValues;

	double MaxExtraErrs, TotalExtraErrs; Tree* XT;
	int NXT;
	float MinCC;
	bool RecalculateErrs;
	float Coeff;
	char* PropVal;
	char *Unquoted;
	int	PropValSize;
	float* AttImp;
	bool *Split, *Used;

	char* Buff;			/* buffer for input characters */
	int	BuffSize, BN;		/* size and index of next character */
	EltRec* TStack;		/* expression stack model */
	int	TStackSize, TSN;	/* size of stack and index of next entry */
	int	DefSize, DN;		/* size of definition and next element */
	bool PreviousError;		/* to avoid parasitic errors */
	AttValue _UNK,			/* quasi-constant for unknown value */
	 _NA;			/* ditto for not applicable */

	#define	MAXLINEBUFFER	10000
	int	Delimiter;
	char	LineBuffer[MAXLINEBUFFER], *LBp;

	CaseNo	SampleFrom;		/* file count for sampling */
	bool SuppressErrorMessages;

	float		ValThresh;	/* minimum GR when evaluating sampled atts */
	bool		Sampled;	/* true if sampling used */
	Attribute	*Waiting,	/* attribute wait list */
				NWaiting;
	bool		MultiVal,	/* all atts have many values */
				Subsample;	/* use subsampling */
	float		AvGainWt,	/* weight of average gain in gain threshold */
				MDLWt;		/* weight of MDL threshold ditto */
	Attribute	*DList;	/* list of discrete atts */
	int			NDList;		/* number in list */
	DiscrValue	MaxLeaves;	/* target maximum tree size */

	double	*Errors,		/* [Condition] */
			*Total;		/* [Condition] */
	float	*Pessimistic,	/* [Condition] */
			*CondCost;		/* [Condition] */
	bool	**CondFailedBy,	/* [Condition][CaseNo] */
			*Deleted;		/* [Condition] */
	Condition	*Stack;
	int		MaxDepth,		/* depth of tree */
			NCond,
			Bestd;
	ClassNo		TargetClass;
	short	*NFail,		/* NFail[i] = conditions failed by i */
			*LocalNFail;	/* copy used during rule pruning */
	CaseNo	Fail0,
			Fail1,
			FailMany,
			*Succ;		/* case following case i */


	///////////////////////////////
	//Formerly global variables for reading in trees
	///////////////////////////////
	bool RULESUSED;
	RuleNo* RulesUsed;
	String OptArg, Option;
	int TotalRules;

/*************************************************************************/
/*									 */
/*		Function prototypes					 */
/*									 */
/*************************************************************************/
	
	void* Pmalloc(size_t Bytes);
	void* Prealloc(void *Present, size_t Bytes);
	void* Pcalloc(size_t Number, unsigned int Size);
	
	void FindLeaf(DataRec Case, Tree T, Tree PT, float Wt) const;
	void FindLeaf(DataRec Case, Tree T, Tree PT, float Wt, float*) const;
	void Quicksort(CaseNo Fp, CaseNo Lp, Attribute Att);
	void Cachesort(CaseNo Fp, CaseNo Lp, SortRec *SRec);
	void FindDepth(Tree T);
	void Show(Tree T, int Sh);
	void ShowBranch(int Sh, Tree T, DiscrValue v, DiscrValue BrNo);
	void Indent(int Sh, int BrNo);
	void Sprout(Tree T, DiscrValue Branches);
	void UnSprout(Tree T);
	void FreeVector(void **V, int First, int Last);
	void FreeCases(void);
	void FreeLastCase(DataRec Case);
	void ResetKR(int KRInit);
	void DayToDate(int DI, String Date);
	void SecsToTime(int Secs, String Time);
	void SetTSBase(int y);
	void CValToStr(ContValue CV, Attribute Att, String DS);
	void PrintConfusionMatrix(CaseNo *ConfusionMat);
	void PrintErrorBreakdown(CaseNo *ConfusionMat);
	void PrintUsageInfo(CaseNo *Usage);
	void Scan(Tree T);
	void SetupNCost(void);
	void PushCondition(void);
	void PopCondition(void);
	void PruneRule(Condition Cond[], ClassNo TargetClass);
	void ProcessLists(void);
	void AddToList(CaseNo *List, CaseNo N);
	void DeleteFromList(CaseNo *Before, CaseNo N);
	void Increment(int d, CaseNo i, double *Total, double *Errors);
	void FreeFormRuleData(void);
	void ListSort(int *L, int Fp, int Lp);
	void FreeRule(CRule R);
	void PrintRule(CRule R);
	void PrintCondition(Condition C);
	void SiftRules(float EstErrRate);
	void InvertFires(void);
	void FindTestCodes(void);
	void SetInitialTheory(void);
	void CoverClass(ClassNo Target);
	void HillClimb(void);
	void InitialiseVotes(void);
	void CountVotes(CaseNo i);
	void UpdateDeltaErrs(CaseNo i, double Delta, RuleNo Toggle);
	void PruneSubsets(void);
	void SetDefaultClass(void);
	void SwapRule(RuleNo A, RuleNo B);
	void OrderRules(void);
	void GenerateLogs(int MaxN);
	void FreeSiftRuleData(void);
	void SetTestIndex(Condition C);
	void Prepare(void);
	void Shuffle(int *Vec);
	void Summary(void);
	void WriteFilePrefix(String Extension);
	void ReadFilePrefix(String Extension);
	void SaveDiscreteNames(void);
	void OutTree(Tree T);
	void AsciiOut(String Pre, String S);
	void InitialiseWeights(void);
	void SetAvCWt(void);
	void EvaluateSingle(int Flags);
	void EvaluateBoost(int Flags);
	void RecordAttUsage(DataRec Case, int *Usage);
	void FreeClassifier(int Trial);
	void ExplicitAtt(FILE *Nf);
	void ListAttsUsed(void);
	void FreeNames(void);
	void ImplicitAtt(FILE *Nf);
	void ReadDefinition(FILE *f);
	void Append(char c);
	void DefSyntaxError(String Msg);
	void DefSemanticsError(int Fi, String Msg, int OpCode);
	void Dump(char OpCode, ContValue F, String S, int Fi);
	void DumpOp(char OpCode, int Fi);
	void ScanTree(Tree T, bool *Used);
	void FreeTreeData(void);
	void SetMinGainThresh(void);
	void FormTree(CaseNo, CaseNo, int, Tree *);
	void SampleEstimate(CaseNo Fp, CaseNo Lp, CaseCount Cases);
	void Sample(CaseNo Fp, CaseNo Lp, CaseNo N);
	void ProcessQueue(CaseNo WFp, CaseNo WLp, CaseCount WCases);
	void EvalDiscrSplit(Attribute Att, CaseCount Cases);
	void DiscreteTest(Tree Node, Attribute Att);
	void FindClassFreq(double [], CaseNo, CaseNo);
	void FindAllFreq(CaseNo, CaseNo);
	void Divide(Tree Node, CaseNo Fp, CaseNo Lp, int Level);
	void EvalDiscreteAtt(Attribute Att, CaseCount Cases);
	void EvalOrderedAtt(Attribute Att, CaseCount Cases);
	void SetDiscrFreq(Attribute Att);
	void EvalContinuousAtt(Attribute Att, CaseNo Fp, CaseNo Lp);
	void EstimateMaxGR(Attribute Att, CaseNo Fp, CaseNo Lp);
	void PrepareForContin(Attribute Att, CaseNo Fp, CaseNo Lp);
	void ContinTest(Tree Node, Attribute Att);
	void AdjustAllThresholds(Tree T);
	void AdjustThresholds(Tree T, Attribute Att, CaseNo *Ep);
	void PrintDistribution(Attribute Att, DiscrValue MinVal, DiscrValue MaxVal, double **Freq, double *ValFreq, bool ShowNames);
	void InitialiseBellNumbers(void);
	void EvalSubset(Attribute Att, CaseCount Cases);
	void Merge(DiscrValue x, DiscrValue y, CaseCount Cases);
	void EvaluatePair(DiscrValue x, DiscrValue y, CaseCount Cases);
	void PrintSubset(Attribute Att, Set Ss);
	void SubsetTest(Tree Node, Attribute Att);
	void AddBlock(DiscrValue V1, DiscrValue V2);
	void MoveBlock(DiscrValue V1, DiscrValue V2);
	void Prune(Tree T);
	void EstimateErrs(Tree T, CaseNo Fp, CaseNo Lp, int Sh, int Flags);
	void GlobalPrune(Tree T);
	void FindMinCC(Tree T);
	void InsertParents(Tree T, Tree P);
	void CheckSubsets(Tree T, bool);
	void InitialiseExtraErrs(void);
	void RestoreDistribs(Tree T, float&);
	void CompressBranches(Tree T);
	void SetGlobalUnitWeights(int LocalFlag);
	void SoftenThresh(Tree T);
	void ResubErrs(Tree T, CaseNo Fp, CaseNo Lp);
	void FindBounds(Tree T, CaseNo Fp, CaseNo Lp);
	void FollowAllBranches(DataRec Case, Tree T, float Fraction) const;
	void FollowAllBranches(DataRec Case, Tree T, float Fraction, float*) const;
	void CheckActiveSpace(int N, RuleNo* Active, int& NActive);
	void MarkActive(RuleTree RT, DataRec Case, RuleNo* Active, int& NActive);
	void SortActive(RuleNo*, int&);
	void CheckUtilityBand(int *u, RuleNo r, ClassNo Actual, ClassNo Default, float&);
	void Uncompress(Byte *CL, int *UCL);
	
	bool ReadName(FILE *f, String s, int n, char ColonOpt);
	bool Expression(void);
	bool Conjunct(void);
	bool SExpression(void);
	bool AExpression(void);
	bool Term(void);
	bool Factor(void);
	bool Primary(void);
	bool Atom(void);
	bool Find(String S);
	bool UpdateTStack(char OpCode, ContValue F, String S, int Fi);
	bool SameDistribution(DiscrValue V1, DiscrValue V2);
	bool Matches(CRule R, DataRec Case);
	bool Satisfies(DataRec Case, Condition OneCond);
	bool NewRule(Condition Cond[], int NConds, ClassNo TargetClass, bool *Deleted, CRule Existing, CaseCount Cover, CaseCount Correct, float Prior);
	bool SameRule(RuleNo r, Condition Cond[], int NConds,ClassNo TargetClass);
	
	int Which(String Val, String *List, int First, int Last);
	int InChar(FILE *f);
	int FindOne(String *Alt);
	int StoreIVal(String s);
	int MaxLine(Tree SubTree);
	int	FindOutcome(DataRec Case, Condition OneCond);
	int TreeSize(Tree T);
	int ExpandedLeafCount(Tree T);
	int TreeDepth(Tree T);
	int Denominator(ContValue Val);
	int GetInt(String S, int N);
	int DateToDay(String DS);
	int TimeToSecs(String TS);
	int TStampToMins(String TS);
	int SingleFail(CaseNo i);
	int MessageLength(RuleNo NR, double RuleBits, float Errs);
	int OrderByUtility(void);
	int OrderByClass(void);
	int DesiredOutcome(CRule R, int TI);
	int SelectTest(RuleNo *RR, int RRN, CRule *Rule);
	int ReadProp(char *Delim);

	Attribute FindAttName(void);
	Attribute FindBestAtt(CaseCount Cases);
	Attribute ChooseSplit(CaseNo Fp, CaseNo Lp, CaseCount Cases, bool Sampled);
	
	AttValue EvaluateDef(Definition D, DataRec Case);

	double DiscrKnownBaseInfo(CaseCount KnownCases, DiscrValue MaxVal);
	double ComputeGain(double BaseInfo, float UnknFrac, DiscrValue MaxVal, CaseCount TotalCases);
	double TotalInfo(double V[], DiscrValue MinVal, DiscrValue MaxVal);
	double KRandom(void);
	double ExecTime(void);
	double rint(double v);
	
	float TrialTreeCost(bool FirstTime);
	float ErrCost(Tree T, CaseNo Fp, CaseNo Lp);
	float ExtraErrs(CaseCount N, CaseCount E, ClassNo C);
	float RawExtraErrs(CaseCount N, CaseCount E);
	float Interpolate(Tree T, ContValue Val) const;
	float CondBits(Condition C);
	float SE(float sum, float sumsq, int no);
		
	CaseNo Group(DiscrValue, CaseNo, CaseNo, Tree);
	CaseNo PrepareForScan(CaseNo Lp);

	CaseCount SumWeights(CaseNo, CaseNo);
	CaseCount SumNocostWeights(CaseNo, CaseNo);
		
	ContValue GreatestValueBelow(ContValue Th, CaseNo *Ep);
	
	ClassNo SelectClass(ClassNo Default, bool UseCosts, float&) const;
	ClassNo SelectClass(ClassNo Default, bool UseCosts, float&, float*) const;

	DiscrValue Elements(Attribute Att, Set S, DiscrValue *Last);
	
	Tree Leaf(double *Freq, ClassNo NodeClass, CaseCount Cases, CaseCount Errors);
	Tree InTree(void);

	String CaseLabel(CaseNo N);
	String RemoveQuotes(String S);
	
	CRuleSet FormRules(Tree T);
	CRuleSet InRules(void);
	
	Byte* Compress(int *L);
	
	CaseCount CalculateDeltaErrs(void);
	
	RuleTree GrowRT(RuleNo *RR, int RRN, CRule *Rule);
	
	CRule InRule(void);
	
	Condition InCondition(void);
	
	Set MakeSubset(Attribute Att);

	#ifdef UTF8
	int UTF8CharWidth(unsigned char *U);
	int wcwidth(wchar_t ucs);
	int wcswidth(const wchar_t *pwcs, size_t n);
	#endif

	void ShowRules(int);

public:
	C5Engine(void);
	~C5Engine(void);

	int c5train(int, char *[], int);
	int c5load(char* fileroot, bool useRules, bool showRules);
	int c5batch(void);
	
	void ConstructClassifiers(void);
	void CrossVal(void);
	void Evaluate(int Flags);
	
	void GetNames(FILE* Nf);
	void GetData(FILE* Df, bool Train, bool AllowUnknownClass);
	Tree GetTree(String Extension);
	int GetTrials(void);
	CRuleSet GetRules(String Extension);
	FILE* GetFile(String Extension, String RW);
	
	ClassNo Classify(DataRec Case, float&) const;
	ClassNo BoostClassify(DataRec Case, int MaxTrial, float&) const;
	ClassNo BoostClassify(DataRec Case, int MaxTrial, float&, float*, float*) const;
	ClassNo TreeClassify(DataRec Case, Tree DecisionTree, float&) const;
	ClassNo TreeClassify(DataRec Case, Tree DecisionTree, float&, float*) const;
	DataRec NewCase(void);
	
	void Cleanup(void);

	void Error(int ErrNo, String S1, String S2);
	void Check(float Val, float Low, float High);
	void FreeRules(CRuleSet RS);
	void PrintRules(CRuleSet, String);
	void ConstructRuleTree(CRuleSet RS);
	void FreeRuleTree(RuleTree RT);
	void CheckFile(String Extension, bool Write);
	void SaveTree(Tree T, String Extension);
	void SaveRules(CRuleSet RS, String Extension);
	void ReadHeader(void);
	void StreamIn(String S, int n);
	void NotifyStage(int);
	void Progress(float);
	void FreeData(void);
	void CheckValue(DataRec Case, Attribute Att);
	void GetMCosts(FILE *f);
	void WinnowAtts(void);
	void InitialiseTreeData(void);
	void PrintTree(Tree T, String Title);
	void FreeTree(Tree T);
	void ChangeFileRoot(String Root);
	void PrintHeader(String Title);

	DataRec GetDataRec(FILE *Df, bool Train);
	DataRec StringToRec(char* input);
	
	CaseNo CountData(FILE *Df);
	
	ClassNo RuleClassify(DataRec Case, CRuleSet RS, float&);
	ClassNo BoostRuleClassify(DataRec Case, int MaxTrial, float&);
	ClassNo RClassify(DataRec Case, float&);
	
	Tree CopyTree(Tree T);

	char ProcessOption(int Argc, char **Argv, char *Str);

	//formally local-static nightmares
	char	*LastExt;
	CaseNo *ConfusionMat;
	int    SaveFOLDS;
	int	OptNo;
	double	URD[55];
	float Total_progress, Current;
    int   Twentieth, LastStage;
	ClassNo	*Wrong;
	int	Entry;
};

#endif