# JuPedSim XML Multi-Exit Transition Validation

Dieses Dokument beschreibt ein natives (kein Python) Validierungs-Setup fuer
`jupedsim-cli` mit XML-Szenarien.

## Vergleichsszenarien

- `round_robin_multi_exit_validation.xml`
- `adaptive_multi_exit_validation.xml`

Beide Szenarien sind identisch (Geometrie, Agenten), unterscheiden sich nur in
`<exits mode=\"...\">`.

## XML-Elemente fuer Multi-Exit

```xml
<decision x="20.0" y="15.0" distance="1.0" />

<exits
  mode="adaptive|least_targeted|round_robin|fixed"
  fixed_index="0"
  expected_time_weight="1.0"
  density_weight="1.0"
  queue_weight="0.0"
  switch_penalty="0.0"
  decision_interval="1"
  reconsideration_threshold="0.0">
  <exit weight="1">...</exit>
  <exit weight="1">...</exit>
</exits>
```

Hinweise:

- `<decision .../>` ist Pflicht, wenn `<exits>` verwendet wird.
- `weight` wird nur bei `mode="round_robin"` ausgewertet.
- Bei `mode="fixed"` wird `fixed_index` genutzt.

## Lauf mit nativer CLI

```bash
./build/bin/jupedsim-cli examples/xml/round_robin_multi_exit_validation.xml \
  --out-jsp /tmp/round_robin_multi_exit_validation.jsp

./build/bin/jupedsim-cli examples/xml/adaptive_multi_exit_validation.xml \
  --out-jsp /tmp/adaptive_multi_exit_validation.jsp
```

`jupedsim-cli` gibt am Ende `iterations=...` aus. Diese Zahl kann direkt fuer
einen A/B-Vergleich verwendet werden.

Beispiel aus diesem Repo (Stand dieser Konfiguration):

- `round_robin_multi_exit_validation.xml`: `iterations=6280`
- `adaptive_multi_exit_validation.xml`: `iterations=4927`
