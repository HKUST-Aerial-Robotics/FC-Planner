#include "LKH.h"
#include "CLARIST.h"

/*
 * The MergeWithTourCLARIST function attempts to find a short tour by merging
 * a given tour, T1, with another tour, T2. T1 is given by the Suc pointers
 * of its nodes. T2 is given by the Next pointers of its nodes.
 *
 * Originally programmed by X. Clarist. Adapted for LKH by K. Helsgaun.
 *
 * The merging algorithm may be described as follows: 
 * Let G be the graph consisting of the nodes and the union of the edges of
 * T1 and T2.
 * The algorithm first identifies the nodes of degree 2 in G.
 * It subsequently identifies the components. A component consists of
 * adjacent nodes in G that are not of degree 2. 
 * This base being established, the goal is to determine if a path of T1
 * that is part of a component can be replaced by a path of T2 that is part
 * of the same component, in the case this reduces the length of T1.
 * It may happen that such a replacement does not result in a tour,
 * because sometimes two or more components are related to each other,
 * and the replacement will only result in a tour if those components are
 * replaced all at once. Therefore, the algorithm tries to find the
 * neighboring components in order to fuse them. 
 * The following process is applied to determine if two components C1 and C2
 * may be fused:
 * Departing from node N of C1, let's move along the paths of T2 inside of
 * C1 and along the paths of T1 outside of C1. If N is met again, and only
 * C1 and C2 have been visited, they are fused. After a round of fusion on
 * components, the following validation process is used to determine if the
 * replacement of the paths of T1 by the paths of T2 inside of a component
 * (or a fused component) will result in a tour:
 * Let n1 be the number of accesses to the component, if the paths of T1
 * are followed inside the component and outside of it. 
 * Let n2 be the number of accesses to the component, if the paths of T2
 * are followed inside the component and the paths of T1 outside of it.
 * If n1 = n2, the replacement will result in a tour.
 * Iterations of fusions and validations are performed as many times as
 * possible (the valid components are excluded after each iteration).
 * Finally, the replacement of valid components that reduce the length is
 * made in T1. 
 *
 * Sometimes a shorter tour can be found by switching T1 and T2.
 * The algorithm above is therefore executed again after a switch of
 * T1 and T2, and the shortest tour found is selected.
 */

GainType MergeWithTourCLARIST()
{
    Node *N, *Prev;
    rec *ptcur;
    int len, i;
    GainType Cost1 = 0, Cost2 = 0, OldCost1, OldCost2;
    static GainType BestCost = PLUS_INFINITY;

    if (vecpttra == NULL) {
        int Dim = Dimension > DimensionSaved ? Dimension : DimensionSaved;
        vecpttra = (rec *) malloc((Dim + 1) * sizeof(rec));
        for (i = 1; i <= Dim; i++)
            vecpttra[i].ID = i;
        lnkdif = (double *) malloc((MAXDIFNBR + 1) * sizeof(double));
        lnkgrp = (double *) malloc((MAXDIFNBR + 1) * sizeof(double));
        grp2 = (int *) malloc((MAXDIFNBR + 1) * sizeof(int));
        grp2N = (int *) malloc((MAXDIFNBR + 1) * sizeof(int));
        difact = (int *) malloc((MAXDIFNBR + 2) * sizeof(int));
        difact++;
        diftst1 = (int *) malloc((MAXDIFNBR + 1) * sizeof(int));
        diftst2 = (int *) malloc((MAXDIFNBR + 1) * sizeof(int));
    }

    N = FirstNode;
    ptdeb = &vecpttra[N->Id];
    do {
        ptcur = &vecpttra[N->Id];
        (ptcur->ptN = ptcur->ptbufN = &vecpttra[N->Suc->Id])->ptP = ptcur;
        (ptcur->pt2N = &vecpttra[N->Next->Id])->pt2P = ptcur;
        ptcur->ptbufN->ptbufP = ptcur;
        N->Next->Prev = N->Suc->Pred = N;
        ptcur->len = (C(N, N->Suc) - N->Pi - N->Suc->Pi) / Precision;
        ptcur->len2 = (C(N, N->Next) - N->Pi - N->Next->Pi) / Precision;
        Cost1 += ptcur->len;
        Cost2 += ptcur->len2;
    } while ((N = N->Suc) != FirstNode);

    if (Cost1 == Cost2) {
        N = FirstNode;
        do {
            if (N->Suc != N->Next && N->Suc != N->Prev)
                break;
        } while ((N = N->Suc) != FirstNode);
        if (N == FirstNode &&
            (N->Suc == N->Next || N->Suc == N->Prev))
            return Cost1;
    }
    N = FirstNode;
    do
        N->OldSuc = N->Suc;
    while ((N = N->Suc) != FirstNode);
    OldCost1 = Cost1;
    OldCost2 = Cost2;

    Cost1 += merge_clarist();
    N = FirstNode;
    do {
        ptcur = &vecpttra[N->Id];
        ptcur->ptbufPsaved = 
            Cost1 <= Cost2 || Cost1 < OldCost1 ? ptcur->ptbufP
                                               : &vecpttra[N->Prev->Id];
        ptcur->ptbufNsaved =
            Cost1 <= Cost2 || Cost1 < OldCost1 ? ptcur->ptbufN
                                               : &vecpttra[N->Next->Id];
    } while ((N = N->Suc) != FirstNode);
    do {
        rec *pttmp;
        ptcur = &vecpttra[N->Id];
        pttmp = ptcur->pt2N;
        (ptcur->pt2N = ptcur->ptN)->pt2P = ptcur;
        (ptcur->ptN = ptcur->ptbufN = pttmp)->ptP = ptcur;
        ptcur->ptbufN->ptbufP = ptcur;
        len = ptcur->len;
        ptcur->len = ptcur->len2;
        ptcur->len2 = len;
    } while ((N = N->Suc) != FirstNode);
    BestCost = Cost1 <= Cost2 ? Cost1 : Cost2;
    Cost2 += merge_clarist();
    if (Cost2 < BestCost) {
        Prev = NULL;
        do {
            ptcur = &vecpttra[N->Id];
            N->Suc = &NodeSet[ptcur->ptbufN->ID] != Prev ?
                     &NodeSet[ptcur->ptbufN->ID] :
                     &NodeSet[ptcur->ptbufP->ID];
            N->Suc->Pred = Prev = N;
        } while ((N = N->Suc) != FirstNode);
        BestCost = Cost2;
    } else if (Cost1 == OldCost1 || Cost1 == OldCost2) {
        N = FirstNode;
        do
            (N->Suc = N->OldSuc)->Pred = N;
        while ((N = N->Suc) != FirstNode);
        return OldCost1;
    } else {
        Prev = NULL;
        do {
            ptcur = &vecpttra[N->Id];
            N->Suc = &NodeSet[ptcur->ptbufNsaved->ID] != Prev ?
                     &NodeSet[ptcur->ptbufNsaved->ID] :
                     &NodeSet[ptcur->ptbufPsaved->ID];
            N->Suc->Pred = Prev = N;
        } while ((N = N->Suc) != FirstNode);
        BestCost = Cost1;
    }
    Hash = 0;
    N = FirstNode;
    do
        Hash ^= Rand[N->Id] * Rand[N->Suc->Id];
    while ((N = N->Suc) != FirstNode);
    if (TraceLevel >= 2 && BestCost < OldCost1 && BestCost < OldCost2)
        printff("CLARIST: " GainFormat "\n", BestCost);
    return BestCost;
}

int merge_clarist()
{
    rec *ptcur;
    int i, j;

    reduce_path_tour1();
    tag_all_components();
    if (difnegfnd) {
        reduce_path_tour2();
        for (i = 1; i <= difnbr; i++) {
            grp2[i] = i;
            grp2N[i] = i;
            diftst1[i] = 0;
            lnkgrp[i] = lnkdif[i];
            difact[i] = 0;
        }
        j = 0;
        do {
            fusgrp2 = 0;
            j++;
            ptdebtog = ptdebcom2;
            fuse_components();
            if (j == 1 || fusgrp2)
                validate_components();
            stop = 1;
            if (fusgrp2) {
                for (i = 1; i <= difnbr; i++) {
                    if (!diftst1[grp2[i]]) {
                        stop = 0;
                        break;
                    }
                }
            }
            if (!stop) {
                ptcur = ptdebcom2;
                do
                    ptcur = ptcur->pt21->ptCC;
                while ((ptcur->pt21 == ptcur->pt22 ||
                        diftst1[grp2[ptcur->diftag]]) &&
                       ptcur != ptdebcom2);
                ptdebcom2 = ptcur;
                if (ptcur->pt21 != ptcur->pt22 &&
                    !diftst1[grp2[ptcur->diftag]]) {
                    do {
                        if (ptcur->pt21 == ptcur->pt22 ||
                            diftst1[grp2[ptcur->diftag]]) {
                            pttmp = ptcur->pt21->ptCC;
                            ptcur->ptCC->ptCC = pttmp;
                            pttmp->ptCC = ptcur->ptCC;
                            ptcur = pttmp;
                        } else
                            ptcur = ptcur->pt21->ptCC;
                    } while (ptcur != ptdebcom2);
                }
            }
        } while (!stop);
        totdif = 0;
        for (i = 1; i <= difnbr; i++) {
            if (diftst1[grp2[i]] && lnkgrp[grp2[i]] < 0) {
                difact[i] = 1;
                totdif += lnkdif[i];
            }
        }
        if (totdif < 0) {
            if (valid_tour()) {
                generate_offspring();
                return totdif;
            }
        }
    }
    return 0;
}

void reduce_path_tour1()
{
    rec *ptcur, *ptcom;

    ptcur = ptdeb;
    if (ptcur->pt2N != ptcur->ptN && ptcur->pt2P != ptcur->ptN) {
        do
            ptcur = ptcur->ptN;
        while (ptcur->pt2N != ptcur->ptN && ptcur->pt2P != ptcur->ptN);
    } else {
        while (ptcur->pt2N == ptcur->ptP || ptcur->pt2P == ptcur->ptP)
            ptcur = ptcur->ptP;
    }
    ptdebcom = ptcur;
    do {
        ptcom = ptcur;
        ptcur = ptcur->ptN;
        while (ptcur->pt2N == ptcur->ptN || ptcur->pt2P == ptcur->ptN) {
            ptcur->diftag = -1;
            ptcur = ptcur->ptN;
        }
        ptcur->ptC = ptcom;
        ptcom->ptC = ptcur;
        ptcur->diftag = 0;
        ptcur = ptcur->ptN;
        while (ptcur->pt2N != ptcur->ptN && ptcur->pt2P != ptcur->ptN) {
            ptcur->ptC = NULL;
            ptcur->diftag = 0;
            ptcur = ptcur->ptN;
        }
        ptcur->diftag = 0;
        ptcur->pt1C = ptcom;
    } while (ptcur != ptdebcom);
}

void find_component_extent(rec * ptcur)
{
    ptcur->diftag = difcnt;
    if (ptcur->ptP->diftag != -1)
        lnkcnt1 += ptcur->ptP->len;
    if (ptcur->ptN->diftag != -1)
        lnkcnt1 += ptcur->len;
    if (ptcur->pt2P->diftag != -1)
        lnkcnt2 += ptcur->pt2P->len2;
    if (ptcur->pt2N->diftag != -1)
        lnkcnt2 += ptcur->len2;
    if (ptcur->ptP->diftag == 0)
        find_component_extent(ptcur->ptP);
    if (ptcur->ptN->diftag == 0)
        find_component_extent(ptcur->ptN);
    if (ptcur->pt2P->diftag == 0)
        find_component_extent(ptcur->pt2P);
    if (ptcur->pt2N->diftag == 0)
        find_component_extent(ptcur->pt2N);
}

void tag_one_component(rec * ptcur)
{
    lnkcnt1 = 0;
    lnkcnt2 = 0;
    find_component_extent(ptcur);
    lnkdif[difcnt] = (lnkcnt2 - lnkcnt1) * 0.5;
    if (lnkdif[difcnt] < 0)
        difnegfnd = 1;
    difcnt++;
}

void tag_all_components()
{
    rec *ptcur, *ptcom, *pttmp;
    long long diftag;

    difcnt = 1;
    difnegfnd = 0;
    ptcur = ptdebcom;
    tag_one_component(ptcur);
    do
        ptcur = ptcur->pt1C;
    while (ptcur->diftag != 0 && ptcur != ptdebcom);
    totC21 = 0;
    ptdebcom2 = ptcur;

    if (ptcur->diftag == 0) {
        do {
            if (ptcur->diftag == 0)
                tag_one_component(ptcur);
            ptcom = ptcur;
            diftag = ptcom->diftag;
            do
                ptcur = ptcur->pt1C;
            while (ptcur->diftag == diftag);
            totC21++;
            pttmp = ptcur->ptC;
            pttmp->pt21 = ptcom;
            ptcom->pt21 = pttmp;
            ptcur->ptCC = pttmp;
            pttmp->ptCC = ptcur;
        } while (ptcur != ptdebcom2);
    } else {
        totC21++;
        pttmp = ptdebcom->ptC;
        pttmp->pt21 = ptdebcom;
        ptdebcom->pt21 = pttmp;
        ptdebcom->ptCC = pttmp;
        pttmp->ptCC = ptdebcom;
    }
    difnbr = difcnt - 1;
}

void reduce_path_tour2()
{
    rec *ptcur, *ptcom, *pttmp;
    long long diftag;

    ptcur = ptdebcom2;
    if (ptcur->ptN == ptcur->pt2N || ptcur->ptP == ptcur->pt2N)
        ptcur = ptcur->ptC;
    ptdebcom2 = ptcur;
    do {
        ptcom = ptcur;
        diftag = ptcom->diftag;
        do {
            do
                ptcur = ptcur->pt2N;
            while (ptcur->ptC == NULL);
            ptcur = ptcur->ptC;
        } while (ptcur->diftag == diftag && ptcur != ptdebcom2);
        pttmp = ptcur->ptC;
        pttmp->pt22 = ptcom;
        ptcom->pt22 = pttmp;
    } while (ptcur != ptdebcom2);
}

void fuse_components()
{
    rec *ptcur, *ptcom, *ptcom1;
    int diftag, idxdif, grp2idxdif;
    int unique;

    ptcom = ptcom1 = ptdebtog;
    do {
        idxdif = ptcom1->diftag;
        grp2idxdif = grp2[idxdif];
        ptcur = ptcom1->pt22->ptCC;
        diftag = 0;
        unique = 0;
        do {
            if (grp2[ptcur->diftag] != grp2idxdif) {
                if (grp2[ptcur->diftag] != grp2[diftag]) {
                    if (diftag == 0) {
                        diftag = ptcur->diftag;
                        unique = 1;
                    } else
                        unique = 0;
                }
                ptcur = ptcur->pt21->ptCC;
            } else
                ptcur = ptcur->pt22->ptCC;
        } while ((unique || diftag == 0) && ptcur != ptcom1);
        if (unique && diftag != 0) {
            fusgrp2 = 1;
            lnkgrp[grp2idxdif] += lnkgrp[grp2[diftag]];
            idx2 = diftag;
            grp2[idx2] = grp2idxdif;
            while (grp2N[idx2] != diftag) {
                idx2 = grp2N[idx2];
                grp2[idx2] = grp2idxdif;
            }
            grp2N[idx2] = grp2N[idxdif];
            grp2N[idxdif] = diftag;
        }
        do
            ptcom1 = ptcom1->pt21->ptCC;
        while (grp2[ptcom1->diftag] == grp2idxdif && ptcom1 != ptcom);
    } while (ptcom1 != ptcom);
}

void validate_components()
{
    rec *ptcom, *ptcur;
    rec *ptout[MAXDIFNBR + 1];
    int difcnt[MAXDIFNBR + 1];
    long long grp2diftag;
    int cnt, i;

    ptcur = ptdebcom2;
    do {
        ptcur->ptE = NULL;
        ptcur = ptcur->pt21;
        ptcur->ptE = NULL;
        ptcur = ptcur->ptCC;
    } while (ptcur != ptdebcom2);

    for (i = 1; i <= difnbr; i++) {
        ptout[i] = NULL;
        difcnt[i] = 0;
        diftst2[i] = 0;
    }
    ptcur = ptdebcom2;
    for (i = 1; i <= 2; i++) {
        do {
            grp2diftag = grp2[ptcur->diftag];
            pttmp = ptout[grp2diftag];
            if (pttmp != NULL) {
                ptcur->ptE = pttmp;
                pttmp->ptE = ptcur;
            }
            if (i == 2)
                difcnt[grp2diftag]++;
            ptcur = ptcur->pt21;
            ptout[grp2[ptcur->diftag]] = ptcur;
            ptcur = ptcur->ptCC;
        } while (ptcur != ptdebcom2);
    }
    ptcom = ptdebcom2;
    do {
        if (diftst2[grp2[ptcom->diftag]] == 0) {
            cnt = 0;
            ptcur = ptcom;
            do {
                cnt++;
                ptcur = ptcur->pt22->ptE;
            } while (ptcur != ptcom);
            grp2diftag = grp2[ptcom->diftag];
            diftst2[grp2diftag] = 1;
            if (cnt == difcnt[grp2diftag])
                diftst1[grp2diftag] = 1;
        }
        ptcom = ptcom->pt21->ptCC;
    } while (ptcom != ptdebcom2);
}

int valid_tour()
{
    rec *ptcur;

    cntC2 = 0;
    ptcur = ptdebcom2;
    do {
        if (!difact[ptcur->diftag])
            ptcur = ptcur->pt21->ptC;
        else
            ptcur = ptcur->pt22->ptC;
        cntC2++;
    } while (ptcur != ptdebcom2);
    return cntC2 == totC21;
}

void generate_offspring()
{
    rec *ptcur, *ptlas, *ptlas2, *pt1C;

    if (ptdebcom2->ptP == ptdebcom2->pt2P ||
        ptdebcom2->ptP == ptdebcom2->pt2N)
        ptdebcom2 = ptdebcom2->ptC;
    ptcur = ptdebcom2;
    do {
        if (difact[ptcur->diftag]) {
            ptlas2 = ptcur->pt21->ptC;
            do {
                pt1C = ptcur->pt1C;
                ptlas = pt1C->ptC->ptP;
                do {
                    ptcur->ptbufP = ptcur->pt2P;
                    ptcur->ptbufN = ptcur->pt2N;
                    ptcur->lenbuf = ptcur->len2;
                    ptcur = ptcur->ptP;
                } while (ptcur != ptlas);
                ptcur = pt1C;
            } while (ptcur != ptlas2);
        } else
            ptcur = ptcur->pt21->ptC;
    } while (ptcur != ptdebcom2);
}
