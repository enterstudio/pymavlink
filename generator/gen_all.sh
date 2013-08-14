#!/bin/sh

for xml in ../message_definitions/*.xml; do
     base=$(basename $xml .xml)
    #./mavgen.py --lang=C --wire-protocol=1.0 --output=C/include_v$protocol $xml || exit 1
	./mavgen.py --lang=csharp --wire-protocol=1.0 --output=Csharp/include_v$protocol $xml || exit 1
 done

cp -f python/mavlink_ardupilotmega_v0.9.py ../mavlink.py
cp -f python/mavlink_ardupilotmega_v1.0.py ../mavlinkv10.py

