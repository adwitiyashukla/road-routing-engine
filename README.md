# road-routing-engine

A route planner for road networks, written from scratch in C++20. It loads a road graph,
preprocesses it, and then answers shortest path queries in microseconds instead of the hundreds of
milliseconds a plain Dijkstra search needs. There is a CLI and a small HTTP server with a JSON API.

Seven query algorithms are built in, and every one of them is checked against plain Dijkstra on
random queries:

| algorithm | idea |
| --- | --- |
| `dijkstra` | textbook Dijkstra with a 4-ary heap |
| `bidirectional` | searches forward from the source and backward from the target |
| `astar` | A* with a great circle lower bound |
| `bidirectional-astar` | bidirectional A* with consistent potentials |
| `alt` | A* with landmark and triangle inequality bounds |
| `bidirectional-alt` | bidirectional ALT, the fastest option that needs no hierarchy |
| `ch` | contraction hierarchies, the default when a hierarchy file is present |

## Build

Needs CMake 3.16 and a compiler with C++20. No third party libraries.

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Use

The engine reads road networks as text: the DIMACS format from the 9th DIMACS Implementation
Challenge, where a `.gr` file holds the arcs after a `p sp` header and a `.co` file holds the
coordinates, and the compact layout the current mirror of that data ships, where the header is
`<region> <metric> <nodes> <arcs>` and the arc lines carry no letter and count from zero. Both are
recognized from the header, and both may be gzipped.

The original challenge site is gone, so I used the mirror on
[Zenodo](https://zenodo.org/records/18026780), which republishes the same road networks. The first
two members of its archive are the North-West US graph and its coordinates, so the first 32 MB of
the download are enough:

```
curl -L -r 0-32000000 "https://zenodo.org/api/records/18026780/files/USA_road_network_and_poi.tar.gz/content" -o head.tgz
tar -xf head.tgz --strip-components=1 data/NW-t.graph data/NW-t.coordinates
build/roadroute import --graph NW-t.graph --coords NW-t.coordinates --out nw.rrg
build/roadroute prepare --graph nw.rrg --threads 8 --landmark-count 16
build/roadroute route --graph nw.rrg --from 47.6062,-122.3321 --to 45.5152,-122.6784 --path none
```

That last query is Seattle to Portland. It comes back with weight 2798297, and on this travel time
metric a weight divided by a million is hours, so 2.80 hours. Seattle to Spokane gives 4.46 hours
and Portland to Boise 6.88, which is what a map says too.

Without any download, the generator writes a synthetic road network in the same DIMACS format. It
lays out a street grid with arterials, highways with ramps, one way streets and curved segments, so
the shape is close enough to a real city to be worth timing:

```
build/roadroute generate --out city --nodes 200000 --seed 1
build/roadroute import --graph city.gr --coords city.co --out city.rrg
build/roadroute prepare --graph city.rrg --threads 8
```

`import` drops self loops and duplicate arcs, keeps the largest strongly connected component so
every query has an answer, and renumbers the nodes along a Hilbert curve so that nodes that are
close on the map are close in memory. `prepare` writes `city.rrch` (the hierarchy) and `city.rrl`
(the landmark tables) next to the graph, and the other commands pick them up automatically.

Query one pair, time all of them, or check them against each other:

```
build/roadroute route --graph city.rrg --from-node 10 --to-node 90000 --path polyline
build/roadroute bench --graph city.rrg --queries 1000 --table 128 --threads 8
build/roadroute difftest --graph city.rrg --queries 1000
```

Serve the graph over HTTP:

```
build/roadroute serve --graph city.rrg --port 8080 --threads 8
```

```
GET /health
GET /route?from=39.10,-84.51&to=39.25,-84.30&algorithm=ch&path=polyline
GET /route?from_node=10&to_node=90000&path=none
GET /nearest?point=39.11,-84.52
GET /table?source_nodes=1;2;3&target_nodes=4;5
```

```json
{"algorithm":"ch","from":{"node":16204,"lat":39.099803,"lon":-84.500188},
 "to":{"node":4,"lat":39.249393,"lon":-84.304040},"weight":20573,"reachable":true,
 "settled":135,"query_ms":0.106,"hops":175,"distance_meters":36179.8,"polyline":"wtsmFd}vbO..."}
```

`weight` is in the unit the input uses, so travel time for the `-t` graphs and meters for the `-d`
graphs. `distance_meters` is measured from the coordinates along the returned path, and
`settled` is how many nodes the search took off its queue.

## How it works

**Graph file.** One file holds the arcs in compressed sparse row form plus the coordinates. Every
section carries a CRC32C, and the loader memory maps the file and reads the arrays in place, so
startup does not depend on the graph size. Small sections are always checksummed, large ones only
with `--verify`, and the structure is validated on every load: offsets must be monotone, arc heads
in range and sorted, no self loops, coordinates on the planet. A graph file also carries an
identity hash, and a hierarchy or landmark file built for a different graph is refused instead of
producing wrong routes.

**Contraction hierarchies.** Nodes are contracted one by one, cheapest first, and a shortcut is
added whenever a node sits on the only shortest path between two of its neighbors. The priority is
the usual edge quotient plus hop quotient plus level term. Whether a shortcut is needed is decided
by a witness search, a small Dijkstra from each incoming neighbor that stops at the length of the
path through the node or after `--settle-limit` settled nodes. Preprocessing runs in parallel: each
round picks an independent set of nodes whose priority is a local minimum, contracts all of them at
once, then repairs the priorities of the nodes they touched. Because the batches are independent
sets and the output order follows the contraction rank, the result does not depend on the thread
count, and the hierarchy files from 1, 2 and 4 threads are byte for byte identical.

A query then searches upward from both ends in the hierarchy and meets in the middle, with stall on
demand to cut branches that a cheaper path already dominates. The arcs that are shortcuts keep the
node they skipped, so the full path is recovered by unpacking them recursively.

**Landmarks.** A few nodes are picked by the avoid heuristic: build a shortest path tree, weigh
each subtree by how badly the current landmarks cover it, and take the deepest node of the worst
subtree. The distances to and from every landmark are stored, and the triangle inequality turns
them into a lower bound for A*. Each query uses the few landmarks that give the strongest bound for
that pair. This costs 8 bytes per node per landmark, which is the reason the hierarchy is the
default for large graphs.

**Many to many.** Tables are built with the bucket method: one upward search from every target
fills buckets, one upward search from every source reads them, so an N by N table costs 2N searches
instead of N squared queries.

**Server.** A small HTTP/1.1 server with its own request parser, a thread pool and keep alive. Each
worker leases a querier with its own search state, so the shared graph and hierarchy stay read
only.

## Results

Measured on my laptop, an HP OmniBook 5 with an Intel Core 5 120U, 12 threads and 8 GB of memory,
on Windows 11 with MinGW GCC 15.2 at `-O2`. The graph is the real North-West US road network from
the DIMACS data, 1,089,933 nodes and 2,545,844 arcs on the travel time metric, and every row is the
same 200 random queries from seed 3.

| algorithm | mean ms | median ms | p99 ms | settled nodes | speedup |
| --- | --- | --- | --- | --- | --- |
| dijkstra | 184.3175 | 177.7717 | 393.7508 | 539,493 | 1.0x |
| bidirectional | 133.6720 | 130.6472 | 326.9565 | 356,321 | 1.4x |
| astar | 129.6191 | 99.7899 | 483.5898 | 247,253 | 1.4x |
| bidirectional-astar | 135.0364 | 118.4034 | 498.2432 | 183,463 | 1.4x |
| alt | 17.4791 | 10.7231 | 101.6983 | 70,756 | 10.5x |
| bidirectional-alt | 7.1808 | 2.2895 | 69.0990 | 18,110 | 25.7x |
| ch | 0.0455 | 0.0379 | 0.1997 | 142 | 4051x |

Each row is its own `bench --algorithms <name>` run, so no algorithm pays for another one's memory
traffic. Running them together in one process is also supported and gives different query times for
the same work, which is the first thing under the next heading.

Preprocessing, on the same laptop:

| step | time | result |
| --- | --- | --- |
| import | 1.34 s | 31.9 MB graph file |
| hierarchy, 12 threads | 10.09 s | 2,050,351 shortcuts, 69.2 MB, 76 rounds, 10,187,358 witness searches |
| 16 landmarks | 17.53 s | 133.0 MB |

A 256 by 256 distance table, so 65,536 pairs, takes 0.006 s on 12 threads, which is 0.09
microseconds per pair. The graph and the hierarchy are memory mapped, so a server starts answering
immediately and only the pages it touches are read. The landmark tables cost 8 bytes per node per
landmark, which is why they are the part to drop first on a small machine.

## Things I ran into

**The data outlived its download page.** Halfway through this project the host for the DIMACS
challenge graphs started answering 404 for every path, including pages that search engines still
index. The mirror I found republishes the same graphs in its own layout: no `p sp` header, no `a`
in front of each arc, and node ids counting from zero instead of one. Rather than convert the files
with a throwaway script that would not be in the repo, the reader now recognizes both layouts from
the header line, so either file works as input and the tests cover both.

**The same query is slower when other algorithms share the process.** A hierarchy query on this
graph takes 0.0455 ms in its own run. Put Dijkstra in the same process and it becomes 0.1465 ms,
because Dijkstra's sweep over half a million nodes leaves nothing of the hierarchy in cache. Load
the 133 MB landmark table as well and run ALT between the hierarchy queries, and the mean goes to
5.9565 ms with a p99 of 49 ms, since on an 8 GB laptop walking that table evicts the memory mapped
hierarchy from the page cache and the next query faults it back off the disk. Same 142 settled
nodes in all three, so none of it is the algorithm. The table above therefore measures one
algorithm per run, and the benchmark interleaves the algorithms query by query when several are
selected, after a warmup round, because when they ran one after another the last one in line was
the one that got the throttled clock.

**Nothing was checking four bytes of the file header.** The graph file reserves a 32 bit field in
its header for a future flag. A test that flips every single bit of a written file and expects the
loader to refuse it found that flipping any bit of that field changed nothing, since it was neither
read nor covered by a checksum. A corrupt file that the loader accepts is exactly the thing the
checksums are there to prevent, so the loader now requires the field to be zero. Round trip tests
never find this class of bug: only writing a file, damaging it, and insisting that the reader
complains does.

**A test that expected shortcuts was wrong, not the code.** On a small graph, a path of four nodes
with a long detour around it, the contraction added no shortcuts at all and my test failed. The
order explains it: the priority function first contracts the node whose removal needs no shortcut,
which peels the detour and then the dead ends, and by the time the middle nodes come up their
neighbors are already gone. Zero shortcuts was the right answer and the assertion went.

**Bidirectional A* needs potentials that agree.** The forward and backward searches have to use the
same function, or the stopping rule is wrong and the result can be too long. The fix is the average
potential, half the difference of the forward and backward bounds, and halving integers loses the
remainder. Keys are stored as twice the distance plus the difference instead, so everything stays
in integers, and the stopping rule compares against twice the best distance found.

**Nearest node lookup has to scale longitude.** The k-d tree holds microdegrees, where a degree of
longitude at 39 degrees north is about 78 percent of a degree of latitude. Comparing raw degrees
picks a node that looks closer but is not, so the tree scales longitude differences by the cosine of
the query latitude before comparing. The test for this searches 20 random point sets by brute force
with the real great circle distance and insists the tree returns a node at the same distance.

## Tests

48 cases across 6 binaries. The ones that find things:

- Dijkstra, both bidirectional searches, A*, both ALT variants and the hierarchy are all compared
  against a Floyd Warshall all pairs table on small networks, so the reference is a different
  algorithm rather than another shortest path search.
- Every returned path is walked arc by arc: it has to start at the source, end at the target, use
  arcs that exist in the graph, and sum to the distance the query reported.
- Hierarchy files are built with 1, 2 and 4 threads and have to give the same answers. On the real
  graph the files from 1, 2 and 12 threads came out byte for byte identical, on two machines.
- Every bit of a graph file is flipped in turn and the loader has to refuse the file.
- The gzip reader is checked against stored, fixed and dynamic Huffman blocks, multi member
  streams, truncation at many offsets and single bit flips.
- Both input layouts are parsed, and 14 malformed files between them have to be refused rather
  than silently producing a smaller graph.
- CRC32C is checked against the published check value for `123456789`, and the polyline encoder
  against Google's published example.
- The landmark and great circle bounds are compared against the all pairs table to confirm they
  never exceed the true distance, which is what makes A* exact.
- The server is exercised over a real socket, including four clients at once on a shared engine.

CI builds on Linux with GCC and Clang, macOS and Windows with MSVC, runs the tests under the
address, undefined and thread sanitizers, runs the whole pipeline end to end, and checks
clang-format.

## License

MIT
