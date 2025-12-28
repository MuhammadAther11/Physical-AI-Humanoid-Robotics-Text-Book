import React, {useState} from 'react';
import clsx from 'clsx';
import styles from './SearchBar.module.css';

function SearchBar() {
  const [searchQuery, setSearchQuery] = useState('');
  const [isExpanded, setIsExpanded] = useState(false);

  const handleSearch = (e) => {
    e.preventDefault();
    // In a real implementation, this would trigger the search
    console.log('Searching for:', searchQuery);
  };

  return (
    <div className={clsx('search', styles.search)}>
      <form 
        onSubmit={handleSearch}
        className={clsx('search__form', styles.searchForm, {
          [styles.searchFormExpanded]: isExpanded
        })}
        role="search">
        <input
          type="search"
          placeholder="Search..."
          aria-label="Search"
          className={clsx('search__input', styles.searchInput)}
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          onFocus={() => setIsExpanded(true)}
          onBlur={() => setTimeout(() => setIsExpanded(false), 200)}
        />
        <button
          type="submit"
          className={clsx('search__button', styles.searchButton)}
          aria-label="Submit search">
          <svg
            className={clsx('search__icon', styles.searchIcon)}
            viewBox="0 0 24 24"
            aria-hidden="true">
            <path
              fill="currentColor"
              d="M15.5 14h-.79l-.28-.27A6.471 6.471 0 0 0 16 9.5 6.5 6.5 0 1 0 9.5 16c1.61 0 3.09-.59 4.23-1.57l.27.28v.79l5 4.99L20.49 19l-4.99-5zm-6 0C7.01 14 5 11.99 5 9.5S7.01 5 9.5 5 14 7.01 14 9.5 11.99 14 9.5 14z"
            />
          </svg>
        </button>
      </form>
      
      {isExpanded && (
        <div className={clsx('search__suggestions', styles.searchSuggestions)}>
          <div className={clsx('search__suggestions-header', styles.searchSuggestionsHeader)}>
            <h3 className={clsx('search__suggestions-title', styles.searchSuggestionsTitle)}>
              Quick Search
            </h3>
            <button 
              className={clsx('search__close-button', styles.searchCloseButton)}
              onClick={() => setIsExpanded(false)}
              aria-label="Close search suggestions">
              ×
            </button>
          </div>
          <ul className={clsx('search__suggestions-list', styles.searchSuggestionsList)}>
            <li className={clsx('search__suggestion-item', styles.searchSuggestionItem)}>
              <a href="/docs/category/getting-started" className={clsx('search__suggestion-link', styles.searchSuggestionLink)}>
                Getting Started
              </a>
            </li>
            <li className={clsx('search__suggestion-item', styles.searchSuggestionItem)}>
              <a href="/docs/category/tutorials" className={clsx('search__suggestion-link', styles.searchSuggestionLink)}>
                Tutorials
              </a>
            </li>
            <li className={clsx('search__suggestion-item', styles.searchSuggestionItem)}>
              <a href="/docs/category/api-reference" className={clsx('search__suggestion-link', styles.searchSuggestionLink)}>
                API Reference
              </a>
            </li>
          </ul>
        </div>
      )}
    </div>
  );
}

export default SearchBar;