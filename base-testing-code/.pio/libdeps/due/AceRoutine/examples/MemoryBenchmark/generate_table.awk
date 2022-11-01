#!/usr/bin/gawk -f
#
# Usage: generate_table.sh < ${board}.txt
#
# Takes the file generated by collect.sh and generates an ASCII
# table that can be inserted into the README.md.

BEGIN {
  labels[0] = "Baseline"
  labels[1] = "One Delay Function"
  labels[2] = "Two Delay Functions"
  labels[3] = "One Coroutine (millis)"
  labels[4] = "Two Coroutines (millis)"
  labels[5] = "One Coroutine (micros)"
  labels[6] = "Two Coroutines (micros)"
  labels[7] = "One Coroutine (seconds)"
  labels[8] = "Two Coroutines (seconds)"
  labels[9] = "One Coroutine, Profiler"
  labels[10] = "Two Coroutines, Profiler"
  labels[11] = "Scheduler, One Coroutine (millis)"
  labels[12] = "Scheduler, Two Coroutines (millis)"
  labels[13] = "Scheduler, One Coroutine (micros)"
  labels[14] = "Scheduler, Two Coroutines (micros)"
  labels[15] = "Scheduler, One Coroutine (seconds)"
  labels[16] = "Scheduler, Two Coroutines (seconds)"
  labels[17] = "Scheduler, One Coroutine (setup)"
  labels[18] = "Scheduler, Two Coroutines (setup)"
  labels[19] = "Scheduler, One Coroutine (man setup)"
  labels[20] = "Scheduler, Two Coroutines (man setup)"
  labels[21] = "Scheduler, One Coroutine, Profiler"
  labels[22] = "Scheduler, Two Coroutines, Profiler"
  labels[23] = "Scheduler, LogBinProfiler"
  labels[24] = "Scheduler, LogBinTableRenderer"
  labels[25] = "Scheduler, LogBinJsonRenderer"
  labels[26] = "Blink Function"
  labels[27] = "Blink Coroutine"
  record_index = 0
}
{
  u[record_index]["flash"] = $2
  u[record_index]["ram"] = $4
  record_index++
}
END {
  NUM_ENTRIES = record_index

  base_flash = u[0]["flash"]
  base_ram = u[0]["ram"]
  for (i = 0; i < NR; i++) {
    u[i]["d_flash"] = u[i]["flash"] - base_flash
    u[i]["d_ram"] = u[i]["ram"] - base_ram
  }

  printf("+--------------------------------------------------------------------+\n")
  printf("| functionality                         |  flash/  ram |       delta |\n")
  for (i = 0; i < NUM_ENTRIES; i++) {
    if (labels[i] ~ /^Baseline$/ \
      || labels[i] ~ /^One Delay Function$/ \
      || labels[i] ~ /^One Coroutine \(millis\)$/ \
      || labels[i] ~ /^One Coroutine \(micros\)$/ \
      || labels[i] ~ /^One Coroutine \(seconds\)$/ \
      || labels[i] ~ /^One Coroutine, Profiler$/ \
      || labels[i] ~ /^Scheduler, One Coroutine \(millis\)$/ \
      || labels[i] ~ /^Scheduler, One Coroutine \(micros\)$/ \
      || labels[i] ~ /^Scheduler, One Coroutine \(seconds\)$/ \
      || labels[i] ~ /^Scheduler, One Coroutine \(setup\)$/ \
      || labels[i] ~ /^Scheduler, One Coroutine \(man setup\)$/ \
      || labels[i] ~ /^Scheduler, One Coroutine, Profiler$/ \
      || labels[i] ~ /^Scheduler, LogBinProfiler$/ \
      || labels[i] ~ /^Blink Function$/ \
    ) {
      printf("|---------------------------------------+--------------+-------------|\n")
    }
    printf("| %-37s | %6d/%5d | %5d/%5d |\n",
        labels[i], u[i]["flash"], u[i]["ram"], u[i]["d_flash"], u[i]["d_ram"])
  }
  printf("+--------------------------------------------------------------------+\n")
}
