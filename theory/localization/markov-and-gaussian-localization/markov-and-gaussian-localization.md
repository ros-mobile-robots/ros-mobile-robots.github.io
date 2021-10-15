## Mobile Robot Localization: Markov and Gaussian

Mobile robot localilzation is the problem of determining the pose of a robot relative to a given map of the environment.
It is also refered to as [position estimation](theory/state-estimation/recursive-state-estimation.md).




### Markov Localization

Probabilistic localization algorithms are variants of teh Bayes filter.
Markov localization is just a different name for the Bayes filter applied to the mobile robot localization problem.

<pre id="markov-localization" style="display:hidden;">
    \begin{algorithm}
    \caption{Markov localization}
    \begin{algorithmic}
    \PROCEDURE{MarkovLocalization}{$bel(x_{t-1}), u_t, z_t, m$}
        \FOR{all $x_t$}
            \STATE $\bar{bel}(x_t) = \int p(x_t | u_t, x_{t-1}, m) bel(x_{t-1}) \,d x_{t-1}$
            \STATE $bel(x_t) = \eta p(z_t | x_t, m) \bar{bel}(x_t)$
        \ENDFOR
        \RETURN $bel(x_t)$ 
    \ENDPROCEDURE
    \end{algorithmic}
    \end{algorithm}
</pre>

<script>
    pseudocode.renderElement(document.getElementById("markov-localization"));
</script>

<pre id="quicksort" style="display:hidden;">
    % This quicksort algorithm is extracted from Chapter 7, Introduction to Algorithms (3rd edition)
    \begin{algorithm}
    \caption{Quicksort}
    \begin{algorithmic}
    \PROCEDURE{Quicksort}{$bel(x_{t-1}), u_t, z_t, m$}
        \IF{$p < r$} 
            \STATE $q = $ \CALL{Partition}{$A, p, r$}
            \STATE \CALL{Quicksort}{$A, p, q - 1$}
            \STATE \CALL{Quicksort}{$A, q + 1, r$}
        \ENDIF
    \ENDPROCEDURE
    \PROCEDURE{Partition}{$A, p, r$}
        \STATE $x = A[r]$
        \STATE $i = p - 1$
        \FOR{$j = p$ \TO $r - 1$}
            \IF{$A[j] < x$}
                \STATE $i = i + 1$
                \STATE exchange
                $A[i]$ with $A[j]$
            \ENDIF
            \STATE exchange $A[i]$ with $A[r]$
        \ENDFOR
    \ENDPROCEDURE
    \end{algorithmic}
    \end{algorithm}
</pre>

<script>
    pseudocode.renderElement(document.getElementById("quicksort"));
</script>

