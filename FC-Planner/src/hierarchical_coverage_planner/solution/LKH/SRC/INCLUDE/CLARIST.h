#ifndef _CLARIST_H
#define _CLARIST_H

typedef struct rec rec;

struct rec {
    int ID;
    rec *ptP, *ptN, *pt2P, *pt2N;
    rec *ptC, *pt1C, *pt21, *pt22, *ptCC, *ptE;
    rec *ptbufP, *ptbufN, *ptbufPsaved, *ptbufNsaved;
    double len, lenbuf, len2;
    long long diftag;
};

#define MAXDIFNBR 100000

double lnkcnt1, lnkcnt2, totdif, *lnkdif, *lnkgrp;
int *grp2, *grp2N, *difact, *diftst1, *diftst2;
long long difcnt, totC21, difnbr, fusgrp2, idx2, cntC2;
int difnegfnd, stop;
rec *ptdeb, *ptdebcom, *ptdebcom2, *ptdebtog, *pttmp, *vecpttra;

void find_component_extent(rec * ptcur);
void fuse_components();
void generate_offspring();
int merge_clarist();
void reduce_path_tour1();
void reduce_path_tour2();
void tag_all_components();
void tag_one_component(rec * ptcur);
void validate_components();
int valid_tour();

#endif

