/*
 *  Systeme de Detection de Collision - Essaim Autonome UAV
 
 */

#include <stdio.h>   /* printf, fprintf                        */
#include <stdlib.h>  /* malloc, free, qsort, rand, srand       */
#include <math.h>    /* sqrtf, INFINITY                        */
#include <time.h>    /* clock(), CLOCKS_PER_SEC, time()        */

#define N 10000   /* Nombre total de drones dans l'essaim      */
#define W 15      /* Taille de la fenetre glissante de voisins */
                  /* W=15 suffit car apres tri sur X, les deux */
                  /* drones les plus proches sont toujours     */
                  /* parmi les voisins immediats               */

/* ============================================================
 *  STRUCTURE : Drone
 *  Represente un micro-drone avec son identifiant unique
 *  et ses coordonnees spatiales 3D en metres.
 * ============================================================ */
struct Drone {
    int   id;   
    float x;   
    float y;    
    float z; 
};

/* 
 *  FONCTION : distance
 *  Calcule la distance euclidienne 3D entre deux drones.
 *  Formule : sqrt( (x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2 )
 */
float distance(struct Drone *a, struct Drone *b) {
    float dx = a->x - b->x;   /* difference sur X */
    float dy = a->y - b->y;   /* difference sur Y */
    float dz = a->z - b->z;   /* difference sur Z */
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* 
 *  FONCTION : compareX
 *  Comparateur passe a qsort() pour trier les drones
 *  par ordre croissant de leur coordonnee X.
 *  qsort() requiert : retour < 0, = 0, ou > 0
*/
int compareX(const void *a, const void *b) {
    struct Drone *da = (struct Drone *)a;
    struct Drone *db = (struct Drone *)b;
    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}

/* 
 *  PROGRAMME PRINCIPAL
*/
int main() {

    srand((unsigned)time(NULL));

    /* 
      Allocation dynamique sur le tas (heap)
     *  malloc alloue un bloc continu de N structures Drone.
     *  On verifie toujours le retour (peut echouer).
     */
    struct Drone *essaim = malloc(N * sizeof(struct Drone));
    if (!essaim) {
        fprintf(stderr, "ERREUR : Echec allocation memoire.\n");
        return 1;
    }

    /*
     Remplissage via arithmetique de pointeurs
     *  INTERDIT  : essaim[i].x = ...
     *  AUTORISE  : (essaim + i)->x = ...
     *  (essaim + i) = adresse du i-eme drone en memoire
      */
    for (int i = 0; i < N; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 1000000) / 100.0f;
        (essaim + i)->y  = (float)(rand() % 1000000) / 100.0f;
        (essaim + i)->z  = (float)(rand() % 1000000) / 100.0f;
    }
 // DEBUT CHRONOMETRE : on mesure tri + balayage uniquement //
    clock_t debut = clock();


    /* 
    Tri selon l'axe X  -->  O(n log n)
     *  Apres tri, deux drones proches en 3D seront adjacents
     *  dans le tableau => la fenetre W suffit pour les trouver.
     */
    qsort(essaim, N, sizeof(struct Drone), compareX);

    /* Balayage avec fenetre glissante  -->  O(n)
     *  Chaque drone i est compare avec ses W voisins suivants.
     *  La paire minimale est forcement dans cette fenetre.
     */
    float min_dist = INFINITY;
    int   id1 = -1, id2 = -1;

    for (int i = 0; i < N - 1; i++) {
        int limite = i + W;
        if (limite >= N) limite = N - 1;

        for (int j = i + 1; j <= limite; j++) {
            float d = distance(essaim + i, essaim + j);
            if (d < min_dist) {
                min_dist = d;
                id1 = (essaim + i)->id;
                id2 = (essaim + j)->id;
            }
        }
    }

    // FIN CHRONOMETRE //
    clock_t fin = clock();
    double ms = (double)(fin - debut) / CLOCKS_PER_SEC * 1000.0;


    //Affichage des resultatS//
    printf("  Resultat Detection de Collision UAV   \n");
    printf("  Drone #%-6d  et  Drone #%d\n", id1, id2);
    printf("  Distance minimale = %.4f m\n", min_dist);
    printf("  Temps d'execution : %.3f ms\n", ms);
    // Liberation memoire//
    free(essaim);
    return 0;
}
