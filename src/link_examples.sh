#!/bin/sh

if [ -n "${SESSIONDIR}" ] ; then
   mkdir -p ${SESSIONDIR}/examples
   if [ -d ${SESSIONDIR}/examples ] ; then
      cd ${SESSIONDIR}/examples
      if [ $? -eq 0 ] ; then
         if [ -n "${TOOLDIR}" ] ; then
            lndir ${TOOLDIR}/examples
         fi
      fi
   fi
fi
