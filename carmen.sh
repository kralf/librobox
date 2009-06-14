#! /bin/sh
# CARMEN module start/stop functions

CARMENCONF="/etc/carmen.conf"

[ -e "$CARMENCONF" ] || exit 0
. $CARMENCONF

export CENTRALHOST="$CARMEN_CENTRAL_HOST"

function carmen_start_module()
{
  PIDFILE="/var/run/`basename $1`.pid"
  start-stop-daemon --start --quiet --background --make-pidfile \
    --pidfile "$PIDFILE" --exec "$1 $2"
}

function carmen_stop_module()
{
  PIDFILE="/var/run/`basename $1`.pid"
  [ -e "$PIDFILE" ] || return

  start-stop-daemon --stop --quiet --pidfile "$PIDFILE" --signal 2
  rm -f "$PIDFILE"
}

function carmen_restart_module()
{
  carmen_stop_module "$1"
  carmen_start_module "$1" "$2"
}
