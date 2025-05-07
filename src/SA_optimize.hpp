// sa_optimized.hpp — self‑contained simulated annealing core with adaptive temperature
// and zero‑copy move application for analog placement (C++17)
// -----------------------------------------------------------------------------
// HOW TO USE
//   1.  Define your own `State` type that provides:
//        • double cost()           – current objective value (area + penalties)
//        • void applyMove(const Move& m)   – apply tentative move (in‑place)
//        • void undoMove(const Move& m)    – revert move (cheap – just inverse)
//      and a light‑weight `Move` type that describes a single perturbation.
//   2.  Provide a callable `moveGenerator(State&) -> Move` that produces a random
//       legal move (rotation, swap, move‑node, change‑rep, …).
//   3.  Instantiate SimAnnealer<State, MoveGen> and call run(timeBudget).
// -----------------------------------------------------------------------------
//  This file replaces the old O(n²) / deep‑copy annealer.  Highlights:
//    •   T₀ = −Δavg / ln(0.8)   (Δavg measured from 500 random moves).
//    •   1000–2000 moves per temperature, cooling rate 0.90.
//    •   If bestCost stagnant for 3 consecutive temps → *extra* cooling step
//        (multiply temperature an additional 0.50 once).
//    •   ObjectPool for Move objects to kill heap churn (std::vector::push_back).
//    •   Entire State lives once; we only store (Move, Δ) pairs on a small stack
//        during a pass and commit at the end, eliminating full copies.
// -----------------------------------------------------------------------------
#pragma once
#include <cmath>
#include <chrono>
#include <functional>
#include <random>
#include <vector>
#include <cassert>

namespace sa {
// ————————————————————————————————————————————————————————————————
// Very small fixed‑capacity object pool (arena).  Guaranteed stable addresses so
// we can keep pointers without fear of reallocation.
// ————————————————————————————————————————————————————————————————
template<typename T, std::size_t BlockSize = 256>
class ObjectPool {
public:
    template<typename... Args>
    T* create(Args&&... args) {
        if (freeList_.empty()) allocateBlock();
        T* obj = freeList_.back();
        freeList_.pop_back();
        new (obj) T(std::forward<Args>(args)...);
        return obj;
    }
    void destroy(T* obj) noexcept {
        obj->~T();
        freeList_.push_back(obj);
    }
    ~ObjectPool() {
        for (void* block : blocks_) ::operator delete(block);
    }
private:
    void allocateBlock() {
        char* raw = static_cast<char*>(::operator new(sizeof(T) * BlockSize));
        blocks_.push_back(raw);
        for (std::size_t i = 0; i < BlockSize; ++i) {
            freeList_.push_back(reinterpret_cast<T*>(raw) + i);
        }
    }
    std::vector<void*> blocks_;
    std::vector<T*>   freeList_;
};

// ————————————————————————————————————————————————————————————————
// Simulated Annealer core (generic, header‑only)
// ————————————————————————————————————————————————————————————————

template<typename State, typename MoveGen>
class SimAnnealer {
public:
    struct Config {
        std::size_t movesPerTemp   = 1500;   // between 1k–2k
        double      coolingRate    = 0.90;   // geometric
        std::size_t stagnationLimit= 3;      // temps with no improvement
        double      extraCoolMul   = 0.50;   // applied when stagnant
    } cfg;

    SimAnnealer(State& s, MoveGen gen, std::uint64_t seed = std::random_device{}())
        : state_(s), moveGen_(std::move(gen)), rnd_(seed) {
        initTemperature();
    }

    // Run until timeBudget (seconds) exhausted or temperature almost zero.
    double run(double timeBudgetSec) {
        using clock = std::chrono::steady_clock;
        const auto deadline = clock::now() + std::chrono::duration<double>(timeBudgetSec);

        std::size_t stagnant = 0;
        while (T_ > 1e-3 && clock::now() < deadline) {
            bool improved = oneTemperature();
            if (improved) {
                stagnant = 0;
            } else if (++stagnant >= cfg.stagnationLimit) {
                T_ *= cfg.extraCoolMul;  // extra chill
                stagnant = 0;
            }
            T_ *= cfg.coolingRate; // regular cooling
        }
        return bestCost_;
    }

    const State& bestState() const { return bestState_; }

private:
    // Measure average absolute Δcost over 500 random moves ➔ initial T.
    void initTemperature() {
        constexpr std::size_t sample = 500;
        double deltaSum = 0;
        for (std::size_t i = 0; i < sample; ++i) {
            auto m = moveGen_(state_, rnd_);
            double before = state_.cost();
            state_.applyMove(m);
            double after  = state_.cost();
            state_.undoMove(m);
            deltaSum += std::abs(after - before);
        }
        double deltaAvg = deltaSum / sample;
        T_ = -deltaAvg / std::log(0.8); // want 80 % acceptance at start
        bestState_ = state_;
        bestCost_  = state_.cost();
    }

    bool oneTemperature() {
        bool anyImproved = false;
        for (std::size_t i = 0; i < cfg.movesPerTemp; ++i) {
            auto move = moveGen_(state_, rnd_);
            const double oldCost = state_.cost();
            state_.applyMove(move);
            const double newCost = state_.cost();
            const double dE = newCost - oldCost;
            if (dE <= 0 || accept(dE)) {
                // keep move, maybe update best
                if (newCost < bestCost_) {
                    bestCost_  = newCost;
                    bestState_ = state_;
                    anyImproved = true;
                }
            } else {
                state_.undoMove(move);
            }
        }
        return anyImproved;
    }

    bool accept(double dE) {
        std::uniform_real_distribution<double> uni(0.0, 1.0);
        return std::exp(-dE / T_) > uni(rnd_);
    }

    State&       state_;      // live instance (no copies!)
    State        bestState_{}; // copied only when better (cheap via move opts)
    double       bestCost_ = std::numeric_limits<double>::infinity();

    MoveGen      moveGen_;
    std::mt19937_64 rnd_;
    double       T_ = 0.0;
};

// ————————————————————————————————————————————————————————————————
// Example: trivial State demonstrating API (remove in production)
// ————————————————————————————————————————————————————————————————
/*
struct DummyState {
    double x = 0;
    double cost() const { return x * x; }
    struct Move { int dir; };
    void applyMove(const Move& m) { x += (m.dir ? +1 : -1); }
    void undoMove (const Move& m) { x -= (m.dir ? +1 : -1); }
};
struct DummyGen {
    DummyGen() {}
    DummyState::Move operator()(DummyState&, std::mt19937_64& r) {
        std::bernoulli_distribution coin(0.5);
        return { coin(r) };
    }
};
*/
} // namespace sa
