# Failed planning strategies with timeout 240s with 100 nodes [combined pddl]

* 'let(hff, ff(axioms=approximate_negative_cycles),eager_greedy([hff], preferred=[hff]))'
* 'let(hadd, add(axioms=approximate_negative_cycles),eager_greedy([hadd], preferred=[hadd]))'
* 'let(hgc, goalcount(),let(hff, ff(axioms=approximate_negative_cycles),eager_greedy([hgc, hff], preferred=[hff])))'
* 'let(hgc, goalcount(),let(hadd, add(axioms=approximate_negative_cycles),eager_greedy([hgc, hadd], preferred=[hadd])))'
* 'let(hff, ff(axioms=approximate_negative_cycles),let(hlm, landmark_sum(lm_rhw(), pref=true, transform=adapt_costs(one), axioms=approximate_negative_cycles),eager_greedy([hff, hlm], preferred=[hff, hlm])))'
* 'let(hff, ff(axioms=approximate_negative_cycles),eager_wastar([hff], preferred=[hff], w=3))'
* 'let(hff, ff(axioms=approximate_negative_cycles),eager_wastar([hff], preferred=[hff], w=5))'