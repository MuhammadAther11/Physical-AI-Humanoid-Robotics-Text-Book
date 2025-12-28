import React from 'react';
import clsx from 'clsx';

const Heading = ({as: As, id, className, ...props}) => {
  if (id) {
    return (
      <As
        {...props}
        id={id}
        className={clsx(className, 'anchorified')}
      />
    );
  }
  return <As {...props} className={className} />;
};

export default Heading;