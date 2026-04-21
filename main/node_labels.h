#ifndef NODE_LABELS_H
#define NODE_LABELS_H

#include <stddef.h>

typedef struct {
    const char *mac;
    const char *id;
} node_label_t;

/*
 * Tabla de identificación de nodos.
 *
 * mac: MAC sin dos puntos, en minúsculas.
 * id : nombre fácil de entender.
 */
static const node_label_t NODE_LABELS[] = {
    { "840d8e377928", "nodo_bme_rele_1" },
    { "24dcc38e2f48", "root_gateway" }
};

static const size_t NODE_LABELS_COUNT =
    sizeof(NODE_LABELS) / sizeof(NODE_LABELS[0]);

#endif

