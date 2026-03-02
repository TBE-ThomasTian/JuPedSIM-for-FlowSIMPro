# JuPedSim XML Multi-Exit Transition Validation

Dieses Dokument beschreibt ein natives (kein Python) Validierungs-Setup fuer
`jupedsim` mit XML-Szenarien.

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
./build/bin/jupedsim examples/xml/round_robin_multi_exit_validation.xml \
  --out-jsp /tmp/round_robin_multi_exit_validation.jsp

./build/bin/jupedsim examples/xml/adaptive_multi_exit_validation.xml \
  --out-jsp /tmp/adaptive_multi_exit_validation.jsp
```

`jupedsim` gibt am Ende `iterations=...` aus. Diese Zahl kann direkt fuer
einen A/B-Vergleich verwendet werden.

Beispiel aus diesem Repo (Stand dieser Konfiguration):

- `round_robin_multi_exit_validation.xml`: `iterations=6280`
- `adaptive_multi_exit_validation.xml`: `iterations=4927`

## Monte-Carlo Unsicherheitsanalyse (P50/P95)

Fuer automatisierte Unsicherheitsauswertung (mehrere Seeds + KPI-Report mit
P50/P95, Exit-Throughput und Hotspots):

```bash
python3 scripts/evac_uncertainty_cli.py \
  --scenario examples/xml/adaptive_multi_exit_validation.xml \
  --cli build/bin/jupedsim \
  --runs 20 \
  --output-dir artifacts/evac_uncertainty
```

Erzeugte Reports:

- `artifacts/evac_uncertainty/summary.json`
- `artifacts/evac_uncertainty/summary.md`
