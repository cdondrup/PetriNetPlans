%module pnpgenerator
 %{
 /* Includes the header in the wrapper code */
 #include "pnpgenerator.h"
 %}
 
 /* Parse the header file to generate wrappers */
 %include "pnpgenerator.h"
