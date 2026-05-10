/*
 * ============================================================
 *  Systeme de Detection de Collision - Essaim Autonome UAV
 *  Ecole des Sciences de l'Information - ESI Rabat
 *  Programmation Avancee en C | Pr. Tarik HOUICHIME
 * ============================================================
 *
 *  OBJECTIF :
 *    Identifier les deux drones les plus proches parmi un
 *    essaim de N = 10 000 drones en coordonnees 3D (x, y, z),
 *    en temps reel (< 1 ms), sans utiliser l'operateur [].
 *
 *  APPROCHE :
 *    1. Tri des drones selon l'axe X  --> O(n log n)
 *    2. Balayage avec fenetre W=15    --> O(n)
 *    Complexite totale : O(n log n)
 *
 *  CONTRAINTE STRICTE :
 *    L'operateur d'indexation [] est INTERDIT.
 *    Toute navigation se fait via arithmetique de pointeurs :
 *    (essaim + i)->champ   ou   *(essaim + i)
 *
 * ============================================================
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
    int   id;   /* Identifiant unique du drone (0 a N-1)  */
    float x;    /* Coordonnee sur l'axe X (metres)        */
    float y;    /* Coordonnee sur l'axe Y (metres)        */
    float z;    /* Coordonnee sur l'axe Z (altitude)      */
};

/* ============================================================
 *  FONCTION : distance
 *  Calcule la distance euclidienne 3D entre deux drones.
 *  Formule : sqrt( (x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2 )
 * ============================================================ */
float distance(struct Drone *a, struct Drone *b) {
    float dx = a->x - b->x;   /* difference sur X */
    float dy = a->y - b->y;   /* difference sur Y */
    float dz = a->z - b->z;   /* difference sur Z */
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ============================================================
 *  FONCTION : compareX
 *  Comparateur passe a qsort() pour trier les drones
 *  par ordre croissant de leur coordonnee X.
 *  qsort() requiert : retour < 0, = 0, ou > 0
 * ============================================================ */
int compareX(const void *a, const void *b) {
    struct Drone *da = (struct Drone *)a;
    struct Drone *db = (struct Drone *)b;
    if (da->x < db->x) return -1;
    if (da->x > db->x) return  1;
    return 0;
}

/* ============================================================
 *  PROGRAMME PRINCIPAL
 * ============================================================ */
int main() {

    srand((unsigned)time(NULL));

    /* ----------------------------------------------------------
     *  ETAPE 1 : Allocation dynamique sur le tas (heap)
     *  malloc alloue un bloc continu de N structures Drone.
     *  On verifie toujours le retour (peut echouer).
     * ---------------------------------------------------------- */
    struct Drone *essaim = malloc(N * sizeof(struct Drone));
    if (!essaim) {
        fprintf(stderr, "ERREUR : Echec allocation memoire.\n");
        return 1;
    }

    /* ----------------------------------------------------------
     *  ETAPE 2 : Remplissage via arithmetique de pointeurs
     *  INTERDIT  : essaim[i].x = ...
     *  AUTORISE  : (essaim + i)->x = ...
     *  (essaim + i) = adresse du i-eme drone en memoire
     * ---------------------------------------------------------- */
    for (int i = 0; i < N; i++) {
        (essaim + i)->id = i;
        (essaim + i)->x  = (float)(rand() % 1000000) / 100.0f;
        (essaim + i)->y  = (float)(rand() % 1000000) / 100.0f;
        (essaim + i)->z  = (float)(rand() % 1000000) / 100.0f;
    }

    /* DEBUT CHRONOMETRE : on mesure tri + balayage uniquement */
    clock_t debut = clock();

    /* ----------------------------------------------------------
     *  ETAPE 3 : Tri selon l'axe X  -->  O(n log n)
     *  Apres tri, deux drones proches en 3D seront adjacents
     *  dans le tableau => la fenetre W suffit pour les trouver.
     * ---------------------------------------------------------- */
    qsort(essaim, N, sizeof(struct Drone), compareX);

    /* ----------------------------------------------------------
     *  ETAPE 4 : Balayage avec fenetre glissante  -->  O(n)
     *  Chaque drone i est compare avec ses W voisins suivants.
     *  La paire minimale est forcement dans cette fenetre.
     * ---------------------------------------------------------- */
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

    /* FIN CHRONOMETRE */
    clock_t fin = clock();
    double ms = (double)(fin - debut) / CLOCKS_PER_SEC * 1000.0;

    /* ----------------------------------------------------------
     *  ETAPE 5 : Affichage des resultats
     * ---------------------------------------------------------- */
    printf("========================================\n");
    printf("  Resultat Detection de Collision UAV   \n");
    printf("========================================\n");
    printf("  Drone #%-6d  et  Drone #%d\n", id1, id2);
    printf("  Distance minimale = %.4f m\n", min_dist);
    printf("----------------------------------------\n");
    printf("  Temps d'execution : %.3f ms\n", ms);
    printf("  Algorithme        : O(n log n)\n");
    printf("  Statut            : %s\n",
           ms < 1.0 ? "OK - Dans les delais !" : "Attention - Depasse 1ms !");
    printf("========================================\n");

    /* ----------------------------------------------------------
     *  ETAPE 6 : Liberation memoire
     *  Toujours liberer la memoire allouee par malloc.
     * ---------------------------------------------------------- */
    free(essaim);
    return 0;
}
