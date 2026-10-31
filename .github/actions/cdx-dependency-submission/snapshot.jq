# Convert the CycloneDX SBoM into a GitHub dependency submission snapshot:
# https://docs.github.com/en/rest/dependency-graph/dependency-submission
#
# Only components carrying a purl are submitted, as the dependency graph
# identifies packages by it. Components the main component depends on are
# direct, anything reached only through them is indirect. Build tools listed
# under formulation are not linked into the firmware and are left out.
#
# Arguments: manifest, sha, ref, correlator, run_id, url
.metadata.component["bom-ref"] as $root
| ([.components[] | select(.purl) | {key: .["bom-ref"], value: .purl}] | from_entries) as $purls
| ([.dependencies[]? | {key: .ref, value: (.dependsOn // [])}] | from_entries) as $deps
| ($deps[$root] // []) as $direct
| {
    version: 0,
    sha: $sha,
    ref: $ref,
    job: {correlator: $correlator, id: $run_id},
    detector: {name: "cdx-dependency-submission", version: .specVersion, url: $url},
    scanned: .metadata.timestamp,
    manifests: {
      ($manifest): {
        name: $manifest,
        file: {source_location: $manifest},
        resolved: ([.components[] | select(.purl) | {
          key: .["bom-ref"],
          value: {
            package_url: .purl,
            relationship: (if (.["bom-ref"] | IN($direct[])) then "direct" else "indirect" end),
            scope: "runtime",
            dependencies: [($deps[.["bom-ref"]] // [])[] | $purls[.] | select(.)]
          }
        }] | from_entries)
      }
    }
  }
