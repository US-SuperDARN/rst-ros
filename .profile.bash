# .profile.bash
# =============

export OSTYPE="linux"
export SYSTEM="linux"
export VISUAL="vim"
export EDITOR="$VISUAL"

if [ -z "${RSTPATH}" ] ; then
  export RSTPATH="/home/radar_user/repos/rst-ros"
fi

. $RSTPATH/.profile/ros.bash
. $RSTPATH/.profile/rst.bash
. $RSTPATH/.profile/base.bash
. $RSTPATH/.profile/general.bash
. $RSTPATH/.profile/superdarn.bash

. $RSTPATH/.profile/idl.bash

