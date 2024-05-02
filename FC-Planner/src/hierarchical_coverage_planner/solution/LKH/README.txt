LKH is an implementation of the Lin-Kernighan traveling salesman heuristic.

The code is distributed for research use. The author reserves all rights to 
the code.


INSTRUCTIONS FOR INSTALLATION: (Version 2.0.10 - October 2022)
-----------------------------

The software is available in gzipped tar format:

	LKH-2.0.10.tgz	(approximately 1.5 MB)

Download the software and execute the following UNIX commands:

  	tar xvfz LKH-2.0.10.tgz
   	cd LKH-2.0.10
	make

An executable file called LKH will now be available in the directory
LKH-2.0.10.

To test the installation run the program by typing ./LKH pr2392.par. 
Then press return. The program should now solve a problem with 2392 nodes.

By typing ./LKH E3k.0.par, the program should solve an instance with 3162
nodes.

For testing the installation on a larger problem, type ./LKH xray14012_1.par.
Then press return. The program should solve an X-ray crystallography instance
with 14012 nodes.

For further instructions, see the user guide and the short description of
their parameters in the DOC directory.

A two-level tree is used as the default tour representation. 
A three-level tree representation may be used instead by compiling the
source code with the compiler option 

	-DTHREE_LEVEL_TREE

Just edit the first line in SRC/Makefile and execute the commands

	make clean
	make

CHANGES IN VERSION 2.0.10:
--------------------------

Tours may now be recombined by Xavier Clarist's recombination (CLARIST).

CLARIST recombination is used by giving the following specification in the parameter file

	RECOMBINATION = CLARIST


CHANGES IN VERSION 2.0.9:
-------------------------

Candidate sets may now be created by means of POPMUSIC by giving the following
specification in the parameter file for LKH:

	CANDIDATE_SET_TYPE = POPMUSIC

The value of the parameter MAX_CANDIDATES is used to trim the candidate set.
There are, however, some other POPMUSIC related parameters. If not specified,
they will take their default values. These parameters are:

    POPMUSIC_SAMPLE_SIZE = <int>  
    Sample size.
    Default: 10.

    POPMUSIC_SOLUTIONS = <int> 
    Number of solutions to generate.
    Default: 50.

    POPMUSIC_MAX_NEIGHBORS = <int>
    Maximum number of nearest neighbors used as candidates in iterated 3-opt for
    POPMUSIC.
    Default: 5.

    POPMUSIC_TRIALS = <int>
    Number of trials used in iterated 3-opt for POPMUSIC. 
    If the value is zero, the number of trials is the size of the subpath
    to be optimized.
    Default: 1.

    POPMUSIC_INITIAL_TOUR = { YES | NO }
    Specifies whether the first generated POPMUSIC tour is used as
    initial tour for Lin-Kernighan.
    Default: NO.


CHANGES IN VERSION 2.0.8:

Tours may now be recombined by GPX2 (Generalized Partition Crossover 2) 
instead of IPT (Iterative Partial Transcription). 

GPX2 is used by giving the following specification in the parameter file:

	RECOMBINATION = GPX2

The possible settings are:

	RECOMBINATION = { IPT | GPX2 } 

IPT is default.

New edge weight types:

    FLOOR_2D
    FLOOR_3D
    TOR_2D
    TOR_3D

    
CHANGES IN VERSION 2.0.7:
-------------------------

Added an approximate K-center clustering algorithm for tour partitioning
(new keyword: K-CENTER). Improved performance on HCP and HPP instances.


CHANGES IN VERSION 2.0.6:
-------------------------

The replacement strategy (CD/RW) proposed by Lozano, Herrera and Cano for
preserving useful diversity in steady-state genetic algorithms has been added.


CHANGES IN VERSION 2.0.5:
-------------------------

Guibas and Stolfi's implementation of Delaunay triangulation has been replaced
by a faster implementation made by Geoff Leach.


CHANGES IN VERSION 2.0.4:
-------------------------

Added hierarchical compression of subproblems in up to 10 levels (Version 2.0.3
allowed only one level).


CHANGES IN VERSION 2.0.3:
-------------------------
 
A simple genetic algorithm has been added. New keyword: POPULATION_SIZE.
Tours found by the first POPULATION_SIZE runs constitute an initial
population of tours. In each of the remaining runs two tours (parents) 
from the current population is recombined into a new tour (child) using 
a variant of the Edge Recombination Crossover (ERX). The parents are 
chosen with random linear bias towards the best members of the population. 
The child is used as initial tour for the next run. If this run produces 
a tour better than the worst tour of the population, then the resulting
tour replaces the worst tour. Premature convergence is avoided by 
requiring that all tours in the population have different costs.


CHANGES IN VERSION 2.0.2:
-------------------------
 
Minor changes in the code for solving subproblems.

 
CHANGES IN VERSION 2.0.1:
-------------------------
 
Improved distance caching and faster add/delete testing for K-opt moves.
This speeds up the execution by 5-10 percent. 


CHANGES IN VERSION 2.0:
-----------------------

The new version extends the previous one by data structures and algorithms
for solving very large instances, and by facilities for obtaining solutions 
of even higher quality. Many changes have been made. Below is given a short
description of the main features.

1. General K-opt moves
One of the most important means in LKH-2 for obtaining high-quality
solutions is its use of general K-opt moves. In the original version of 
the Lin-Kernighan algorithm moves are restricted to those that can be
decomposed into a 2- or 3-opt move followed by a (possible empty) sequence
of 2-opt moves. This restriction simplifies implementation but it needs not
be the best design choice if high-quality solutions are sought. This has
been demonstrated with LKH-1, which uses a 5-opt sequential move as the
basic move component. LKH-2 takes this idea a step further. Now K-opt moves
can be used as sub-moves, where K is any chosen integer greater than or
equal to 2 and less than the number of cities. Each sub-move is sequential.
However, during the search for such moves, non-sequential moves may also be 
examined. Thus, in contrast to the original version of the Lin-Kernighan 
algorithm, non-sequential moves are not just tried as last resort but are
integrated into the ordinary search.

2. Partitioning
In order to reduce the complexity of solving large-scale problem instances, 
LKH-2 makes it possible to partition a problem into smaller subproblems. 
Each subproblem is solved separately and its solution is used (if possible)
to improve a given overall tour. Even the solution of small problem
instances may sometimes benefit from partitioning as it helps in focusing
in the search process. Currently, LKH-2 implements the following six
partitioning schemes: Tour segment partitioning, Karp partitioning,
Delaunay partitioning, K-means partitioning, Rohe partitioning, and
Sierpinski partitioning.

3. Tour merging
LKH-2 provides a tour merging procedure that attempts to produce the best 
possible tour from two or more given tours using local optimization of an 
instance that includes all tour edges, and where edges common to the tours 
are fixed. Tours that are close to optimal typically share many common
edges. Thus, the input graph for this instance is usually very sparse,
which makes it practicable to use K-opt moves for rather large values of K.

4. Iterative partial transcription
Iterative artial description is a general procedure for improving the 
performance of a local search based heuristic algorithms. It attempts to 
improve two individual solutions by replacing certain parts of either
solution by the related parts of the other solution. The procedure may be
applied to the TSP by searching for subchains of two tours, which contains
the same cities in a different order and have the same initial and final
cities. LKH-2 uses the procedure on each locallyoptimal tour and the up to
now best tour. The implemented algorithm is a simplified version of the
algorithm described by Moebius, Freisleben, Merz and Schreiber.

5. Backbone-guided search
The edges of the tours produced by a fixed number of initial trials may be
used as candidate edges in the succeeding trials. This algorithm, which is
a simplified version of the algorithm given by Zhang and Looks, has turned
out to be particularly effective for VLSI instances.

6. Data structures and algorithms for solving very large instances

Delaunay triangulation may be used to speed up the determination of alpha-
nearest candidate edges, and tours may be represented internally by three-
level trees. 

New keywords:

  BACKTRACKING = { YES | NO }
  CANDIDATE_SET_TYPE = { ALPHA | DELAUNAY [PURE ] | NEAREST-NEIGHBOR | 
                         QUADRANT | REINELT }
  EXTRA_CANDIDATES = <integer> [ SYMMETRIC ]
  EXTRA_CANDIDATE_SET_TYPE = { NEAREST-NEIGHBOR | QUADRANT | REINELT }
  GAIN_CRITERION = { YES | NO }
  INITIAL_TOUR_ALGORITHM = { BORUVKA | GREEDY | NEAREST-NEIGHBOR |
                             QUICK-BORUVKA | SIERPINSKI | WALK }
  INITIAL_TOUR_FRACTION = <real in [0;1]>
  KICKS = <integer>
  KICK_TYPE = <integer>
  MAX_BACKBONE_TRIALS = <integer>
  MAX_BREADTH = <integer>
  MERGE_TOUR_FILE = <string>
  NONSEQUENTIAL_MOVE_TYPE = <integer>
  PATCHING_A = <integer> [ RESTRICTED | EXTENDED ]
  PATCHING_C = <integer> [ RESTRICTED | EXTENDED ]
  SUBPROBLEM_SIZE = <integer> [ DELAUNAY | KARP | K-MEANS | ROHE | MOORE | 
                                SIERPINSKI ]  [ COMPRESSED ] [ BORDERS ]
  SUBPROBLEM_TOUR_FILE = <string>	
  SUBSEQUENT_MOVE_TYPE = <integer>
  SUBSEQUENT_PATCHING = { YES | NO }
  # <string>

Removed keywords:

  BACKTRACK_MOVE_TYPE
  MERGE_TOUR_FILE_1
  MERGE_TOUR_FILE_2


CHANGES IN VERSION 1.3:
----------------------

The distance type GEOM has been added (see www.math.princeton.edu/tsp/world). 
Additional control information may now be given in the parameter file by
means of the following keywords:

  BACKTRACK_MOVE_TYPE
  CANDIDATE_FILE
  INITIAL_TOUR_FILE
  MAX_SWAPS
  MERGE_TOUR_FILE_1
  ERGE_TOUR_FILE_2
  RESTRICTED_SEARCH


CHANGES IN VERSION 1.2:
----------------------

Execution times may be measured more accurately, if the getrusage function
is supported by the system. See the GetTime.c file for instructions.


CHANGES IN VERSION 1.1:
----------------------

The code has been made more robust regarding the solution of asymmetric 
problems. The old code (LKH-1.0, February 1999) could loose its way in some 
cases due to integer overflow.
This directory contains an experimental version of LKH that incorporates
POPMUSIC for TSP.

By executing the command

    make

an executable LKH is made available.

To test the installation, run the program by typing ./LKH pr2392.par.
Then press return. The program should now solve an instance with 2392 nodes.

For testing the installation on a larger problem, type ./LKH xray14464_1.par.
Then press return. The program should solve an X-ray crystallography instance
with 14464 nodes.

A POPMUSIC candidate set is used by giving the following specification in the
parameter file for LKH:

	CANDIDATE_SET_TYPE = POPMUSIC

The value of the parameter MAX_CANDIDATES is used to trim the candidate set.

There are, however, some other POPMUSIC related parameters. If not specified,
they will take their default values. These parameters are:

    POPMUSIC_INITIAL_TOUR = { YES | NO }
    Specifies whether the first generated POPMUSIC tour is used as
    intial tour for Lin-Kernighan.
    Default: NO.

    POPMUSIC_SAMPLE_SIZE = <int>  
    Sample size.
    Default: 10.

    POPMUSIC_SOLUTIONS = <int> 
    Number of solutions to generate.
    Default: 20.

    POPMUSIC_MAX_NEIGHBORS = <int>
    Maximum number of nearest neighbors used as candidates in iterated 3-opt for
    POPMUSIC. 
    Default: 5.

    POPMUSIC_TRIALS = <int>
    Number of trials used in iterated 3-opt for POPMUSIC.
    Default: 1.
