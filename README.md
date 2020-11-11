# Introduction

A hardware implementation of the Quicksort algorithm.

Some years prior, after having been laid-off (a common occurence for
many of us who work in the semiconductor industry), I sat with a
number of ex-colleagues and, over a beer, ruminated about how much
easier life would have been had we chosen a career in software. One of
the individuals at this gathering, a software engineer, lamented that
he had been passed over by Google for not being able to explain the
quicksort algorithm. Quicksort is a fundamental algorithm in computer
science and it is used heavily in just about every aspect of
software. Although, a software engineer may not know precisely how to
implement it in an interview setting, a good understanding at a
high-level is really necessary, particularly when interviewing for a
highly paid position at a company such as Google.

This discussion has stuck with me over the years and I have decided
that it would be interesting to implement the quicksort algorithm in
hardware. Quicksort, as an algorithm, would almost never be
implemented as part of any serious design as there are better
approaches to sorting that can be performed more efficiently in
hardware. Nevertheless, the complexity in its implementation is
noteworthy and interesting from a pedagogical perspective, even if
from a practical standpoint it makes little sense. Also, if any one
from Google is reading this, I would be very interested in a software
position with you!

# Dependencies

The following external dependencies must be satisifed to run the project.

* Verilator (version >= 4.035)
* A compiler supporting C++17

# Basic build instructions

``` shell
# Clone project
git clone git@github.com:stephenry/qs.git
pushd qs
# Check out external dependencies
git submodule init
git submodule update
# Build project
mkdir build
pushd build
# Configure project
cmake ..
# Build project
cmake --build .
# Run regression using the CTEST driver (takes minutes)
ctest .

## Configuration

The Verilator configuration script located in
[FindVerilator.cmake](./cmake/FindVerilator.cmake) expects to discover
a Verilator installation at certain pre-defined paths. If Verilator
has not been installed at one of these known paths, it becomes
necessary to update the 'HINTS' field to point to the appropriate
location on your system.

# Build with VCD

``` shell
# Enable waveform dumping (slows simulation)
cmake -DOPT_VCD_ENABLE=ON ..
```

# Build with logging

``` shell
# Enable logging (slows simulation)
cmake -DOPT_LOGGING_ENABLE=ON ..
```

# Run a test

``` shell
# Run smoke test
./tb/test_tb_qs_smoke

# Run fully randomized regression
./tb/test_tb_qs_regress

# Run all registered tests
cmake .
```

# Discussion

* Sorting is a foundational aspect of many computer algorithms but it
  is rarely performed in hardware. When performed, it is typically
  used for the purpose of loading balancing the allocation of shared
  resources and is carried out using a parallel sorting network. Such
  sort networks utlitise a series of comparions and swaps in order
  parallel. Traditional sort algorithm, such as quicksort, are
  atypical in computer hardware because of their recursive nature and
  the non-trivial complexity in its implementation. 
* Sorting can also be carried out in hardware in O(N) using the
  insertion sort algorithm. In this approach, an associatively
  addressed shift register is used to compute, in parallel, the
  correct location into which an entry is to be placed, after which
  insertion can be carried out in constant time. This approach has
  been used in the "ob" project, which implements a high-performance
  financial matching engine in hardware.
* For the purpose of this demonstrationm, quicksort was chosen because
  of its complex nature and the difficulty one would have when
  attempting to implement the necessary control structures in hardware
  to realize its implementation.
* The implementation of quicksort in hardware is complicated by its
  recursive nature. In quicksort, a number of comparsion are carried
  out so as to partition an array around some value. Was this
  partitioning is carried out, the process is repeated recursively in
  each of these partitions, until their size as fallen to zero
  elements.
* The solution was achieved by implementing a small microcoded
  sequencer which ran the quicksort algorithm. The sequencer consists
  of three pipeline stages and implements a small RISC-like assembly
  language, with specialized instructions to handle synchronization
  between memory banks.
* A DSP-like multibanked arrange is implement to allow data to be
  enqueued and dequeued from the sorted while the sort algorithm is
  taking place. In this case, the advantage of such a scheme is
  perhaps minimal as the overall duration of the sort operation vastly
  exceeds the time taken to load and deload the unsorted and sorted
  states.
* Aside from the complexity of the quicksort algorithm itself when
  implemented in hardware. One must also consider the overhead
  associated with the stack memory which is necessary to maintain the
  recursive state. In actuality, the stack memory required to
  implement quicksort is substantially larger than the actual memory
  being sorted itself. In addition, the ability of the algorithm to
  perform the sort operation across a given vector length, is limited
  by the stack capacity, which grows substantially in relation. This
  difficulty too renders the algorithmic approach somewhat inpractical
  in a serious system.

