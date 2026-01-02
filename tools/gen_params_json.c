#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdalign.h>

#include "../../firmware/Drivers/param_storage.h"

typedef struct {
  const char *name;
  size_t offset;
  const char *type;
  size_t size;
} param_entry_t;

#define PARAM_ENTRY(name, field, type_str) \
  { name, offsetof(robot_params_t, field), type_str, sizeof(((robot_params_t *)0)->field) }

static const param_entry_t kParams[] = {
#include "param_map.def"
};

static size_t type_size(const char *type) {
  if (strcmp(type, "i8") == 0 || strcmp(type, "u8") == 0) {
    return 1;
  }
  if (strcmp(type, "i16") == 0 || strcmp(type, "u16") == 0) {
    return 2;
  }
  if (strcmp(type, "i32") == 0 || strcmp(type, "u32") == 0 || strcmp(type, "float") == 0) {
    return 4;
  }
  return 0;
}

static int compare_offset(const void *a, const void *b) {
  const param_entry_t *pa = (const param_entry_t *)a;
  const param_entry_t *pb = (const param_entry_t *)b;
  if (pa->offset < pb->offset) return -1;
  if (pa->offset > pb->offset) return 1;
  return strcmp(pa->name, pb->name);
}

int main(void) {
  const size_t count = sizeof(kParams) / sizeof(kParams[0]);
  param_entry_t *entries = (param_entry_t *)malloc(sizeof(kParams));
  if (!entries) {
    fprintf(stderr, "alloc failed\n");
    return 1;
  }
  memcpy(entries, kParams, sizeof(kParams));
  qsort(entries, count, sizeof(entries[0]), compare_offset);

  for (size_t i = 0; i < count; ++i) {
    const size_t expected = type_size(entries[i].type);
    if (expected == 0 || entries[i].size != expected) {
      fprintf(stderr, "size mismatch for %s: type=%s size=%zu expected=%zu\n",
              entries[i].name, entries[i].type, entries[i].size, expected);
      free(entries);
      return 1;
    }
  }

  const size_t reserved_offset = offsetof(robot_params_t, reserved);
  const size_t reserved_len = sizeof(((robot_params_t *)0)->reserved);

  printf("{\n");
  printf("  \"version\": %u,\n", (unsigned)PARAM_VERSION);
  printf("  \"struct\": \"robot_params_t\",\n");
  printf("  \"source\": \"firmware/Drivers/param_storage.h\",\n");
  printf("  \"generator\": \"tools/gen_params_json.c\",\n");
  printf("  \"endianness\": \"little\",\n");
  printf("  \"alignment\": %u,\n", (unsigned)alignof(robot_params_t));
  printf("  \"size_bytes\": %zu,\n", sizeof(robot_params_t));
  printf("  \"reserved\": { \"offset\": %zu, \"length\": %zu },\n", reserved_offset, reserved_len);
  printf("  \"notes\": [\n");
  printf("    \"Offsets assume GCC/ARM default alignment for robot_params_t.\",\n");
  printf("    \"Reserved range is omitted from params list.\"\n");
  printf("  ],\n");
  printf("  \"params\": [\n");
  for (size_t i = 0; i < count; ++i) {
    printf("    { \"name\": \"%s\", \"offset\": %zu, \"type\": \"%s\" }%s\n",
           entries[i].name, entries[i].offset, entries[i].type,
           (i + 1 == count) ? "" : ",");
  }
  printf("  ]\n");
  printf("}\n");

  free(entries);
  return 0;
}
