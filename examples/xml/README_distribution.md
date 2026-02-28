# JuPedSim XML Distribution Modes (CLI)

Diese README beschreibt die neuen XML-Optionen in `jupedsim`, um Startpositionen automatisch zu streuen.

## Kurzüberblick

Es gibt jetzt 2 Wege:

1. Explizite Agenten wie bisher:
```xml
<agents>
  <agent x="2.0" y="5.0" desired_speed="1.3" radius="0.2" time_gap="1.0" />
</agents>
```

2. Automatische Verteilung:
```xml
<agents>
  <distribution mode="by_number" number_of_agents="100" distance_to_agents="0.5" />
</agents>
```

Du kannst beides kombinieren: vorhandene `<agent/>` bleiben, Distribution fügt weitere hinzu.

## Unterstützte Modi

- `by_number`
- `by_density`
- `in_circles_by_number`
- `in_circles_by_density`
- `until_filled`
- `by_percentage`

## Gemeinsame Attribute für `<distribution ...>`

- `distance_to_agents` (minimaler Abstand Agent-Zentrum zu Agent-Zentrum, > 0)
- `distance_to_polygon` (minimaler Abstand Agent-Zentrum zu Geometriegrenze, >= 0)
- `seed` (optional, reproduzierbar)
- `max_iterations` (optional, Standard `10000`)
- `k` (nur relevant für `until_filled`/`by_percentage`, optional, Standard `30`)

Hinweis: Die CLI erzwingt intern zusätzlich robuste Mindestabstände basierend auf dem größten `radius` aus den Profilen.

## Modus-spezifische Attribute

### `mode="by_number"`
- `number_of_agents`

### `mode="by_density"`
- `density` (Agenten pro m²)

### `mode="in_circles_by_number"`
- `center_x`, `center_y`
- pro Ring: `<segment min_radius="..." max_radius="..." number_of_agents="..."/>`

### `mode="in_circles_by_density"`
- `center_x`, `center_y`
- pro Ring: `<segment min_radius="..." max_radius="..." density="..."/>`

### `mode="until_filled"`
- keine zusätzlichen Pflichtattribute

### `mode="by_percentage"`
- `percent` im Bereich `(0, 100]`

## Profile für Bewegungsparameter / Altersklassen

Profile können direkt unter `<distribution>` oder unter `<agents>` definiert werden:

```xml
<profile
  desired_speed="1.55"
  radius="0.19"
  time_gap="0.75"
  age_group="young"
  avatar_hint="young"
  weight="1.0" />
```

Attribute:
- `desired_speed`, `radius`, `time_gap`
- `age_group` (`young|adult|elderly`, optional)
- `avatar_hint` (`young|adult|grandpa|grandma`, optional)
- `weight` (Profilgewicht bei Zufallsauswahl, > 0)

Wenn keine Profile angegeben sind, nutzt die CLI ein Default-Profil.

## Position im XML

Beides wird unterstützt:

- `<scenario><agents><distribution .../></agents></scenario>`
- `<scenario><agent_distribution .../></scenario>`

## Beispiel-Dateien

Siehe in diesem Ordner:
- `distribution_by_number.xml`
- `distribution_by_density.xml`
- `distribution_in_circles_by_number.xml`
- `distribution_in_circles_by_density.xml`
- `distribution_until_filled.xml`
- `distribution_by_percentage.xml`

## CLI-Aufruf

```bash
./build/bin/jupedsim examples/xml/distribution_by_number.xml --out-jsp examples/xml/distribution_by_number.jsp
```
