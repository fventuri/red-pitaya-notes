#! /bin/sh

apps_dir=/media/mmcblk0p1/apps

# The ACP/DMA data path hangs if the PL is reprogrammed after the ACP port has been
# active. Once one of the HPSDR2 receivers has loaded its bitstream this boot, refuse
# to program the PL again until a reboot. /tmp is tmpfs, so the flag clears on reboot.
if [ -f /tmp/needs-reboot ]; then
  echo "ERROR: an ACP/DMA FPGA design was already loaded this boot; reprogramming the" >&2
  echo "PL now would hang the board. Reboot before starting this receiver." >&2
  exit 1
fi

. $apps_dir/stop.sh

# create the reboot guard right before loading the bitstream
touch /tmp/needs-reboot

cat $apps_dir/sdr_receiver_hpsdr2_narrow_trx_duo/sdr_receiver_hpsdr2_narrow_trx_duo.bit > /dev/xdevcfg

# --- second virtual HPSDR radio ---------------------------------------------------------
# The FPGA has 16 DDCs but stock HPSDR clients cap the receiver count (linhpsdr 8, Thetis
# 12). The server presents the DDCs as two 8-DDC radios, one per network interface, so a
# stock client sees two independent radios (no patched client needed). Create a macvlan
# interface mvl0 on eth0 with its own MAC for the second radio; its IPv4 is configured by
# dhcpcd via /etc/dhcpcd.conf (which has an `interface mvl0` stanza mirroring eth0).
if ! ip link show mvl0 >/dev/null 2>&1; then
  eth_mac=$(cat /sys/class/net/eth0/address)
  # Derive mvl0's MAC from eth0's: set the locally-administered bit on octet 1 AND toggle a
  # bit in octet 5. CW Skimmer Server identifies a radio by only the last three MAC octets,
  # so differing solely in octet 1 makes the two virtual radios indistinguishable to it.
  # Toggle (XOR), not OR, so the bit flips regardless of its starting value -- an OR leaves
  # octet 5 unchanged when eth0 already has that bit set, which would clone eth0's MAC.
  o1=$(printf '%02x' $(( 0x${eth_mac%%:*} | 0x02 )))
  o5=$(printf '%02x' $(( 0x$(echo "$eth_mac" | cut -d: -f5) ^ 0x02 )))
  o234=$(echo "$eth_mac" | cut -d: -f2-4)
  o6=$(echo "$eth_mac" | cut -d: -f6)
  ip link add mvl0 link eth0 address "$o1:$o234:$o5:$o6" type macvlan mode bridge
fi
# eth0 and mvl0 share one IP subnet; without these a request for one interface's IP could be
# answered with the other interface's MAC (ARP flux), cross-wiring the two radios.
echo 1 > /proc/sys/net/ipv4/conf/all/arp_ignore
echo 2 > /proc/sys/net/ipv4/conf/all/arp_announce
echo 2 > /proc/sys/net/ipv4/conf/all/rp_filter
ip link set mvl0 up
dhcpcd mvl0 2>/dev/null   # apply /etc/dhcpcd.conf (DHCP with static fallback)
# wait (up to ~8 s) for mvl0 to get an IPv4 so the server can advertise it
i=0
while [ $i -lt 40 ]; do
  ip -4 addr show mvl0 2>/dev/null | grep -q 'inet ' && break
  sleep 0.2; i=$((i + 1))
done
ip -4 addr show mvl0 2>/dev/null | grep -q 'inet ' || \
  echo "warning: mvl0 has no IPv4 yet; the second radio may be unreachable (check dhcpcd.conf)" >&2
# ----------------------------------------------------------------------------------------

# Per-DDC ADC assignment for the 16 physical DDCs (radio 0 = DDC0-7, radio 1 = DDC8-15):
#   0 = host chooses the ADC for this DDC (e.g. linhpsdr/Thetis ADC checkbox)  <- default
#   1 = force ADC0    2 = force ADC1
# Set 1/2 to pin a DDC for a client that can't select the ADC (CW Skimmer Server, SparkSDR);
# leave 0 to let the client choose. All 0 (below) = every DDC host-controlled, as before.
$apps_dir/sdr_receiver_hpsdr2_narrow_trx_duo/sdr-receiver-hpsdr2 \
  0 0 0 0 0 0 0 0  0 0 0 0 0 0 0 0 &
