/****************************************************************************
 * apps/examples/modlogsetmask/mod_setmask.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>
#include "modlog_filter.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode){
  printf("\nUsage: %s <e|d|m|c|u|r>\n", progname);
  printf("       %s -h\n", progname);
  printf("\nWhere:\n");
  printf("  e=enable all module log\n");
  printf("  d=disable all module log\n");
  printf("  m=enable/disable motion module log\n");
  printf("  c=enable/disable charger module log \n");
  printf("  u=enable/disable ultrasound emergency stop module log\n");
  printf("  r=enable/disable remote controller module log\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[]){

  if (argc < 2)
    {
      show_usage(argv[0], EXIT_FAILURE);
    }

  switch (*argv[1])
    {
      case 'd':
        {
          printf("turn off all module log\n");
          moduleunmask(LOG_ALL_DISABLED);
        }
        break;
      case 'e':
        {
          printf("turn on all module log\n");
          modulemask(~LOG_ALL_DISABLED);
        }
        break;
      case 'm':
        {
          if(g_module_mask & LOG_MODULE_MASK(LOG_MOTION))
            {
              printf("turn off Motion log\n");
              moduleunmask(~(LOG_MODULE_MASK(LOG_MOTION)));
            }
          else
            {
              printf("turn on Motion log\n");
              modulemask(LOG_MODULE_MASK(LOG_MOTION));
            }
        }
        break;
      case 'c':
        {
          if(g_module_mask & LOG_MODULE_MASK(LOG_CHARGER))
            {
              printf("turn off Charger log\n");
              moduleunmask(~(LOG_MODULE_MASK(LOG_CHARGER)));
            }
          else
            {
              printf("turn on Charger log\n");
              modulemask(LOG_MODULE_MASK(LOG_CHARGER));
            }
        }

        break;
      case 'u':
        {
          if(g_module_mask & LOG_MODULE_MASK(LOG_ULEMERG))
            {
              printf("turn off Ulemerg log\n");
              moduleunmask(~(LOG_MODULE_MASK(LOG_ULEMERG)));
            }
          else
            {
              printf("turn on Ulemerg log\n");
              modulemask(LOG_MODULE_MASK(LOG_ULEMERG));
            }
        }
       break;
      case 'r':
        {
          if(g_module_mask & LOG_MODULE_MASK(LOG_RMCTL))
            {
              printf("turn off remote ctrl log\n");
              moduleunmask(~(LOG_MODULE_MASK(LOG_RMCTL)));
            }
          else
            {
              printf("turn on remote ctrl log\n");
              modulemask(LOG_MODULE_MASK(LOG_RMCTL));
            }
        }
       break;
      default:
        {
          show_usage(argv[0], EXIT_FAILURE);
        }
        break;
    }

  return EXIT_SUCCESS;
}