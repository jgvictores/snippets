<?php
/**
 * Remove unused user accounts from the database
 * An unused account is one which has made no edits
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @file
 * @ingroup Maintenance
 * @author Juan G Victores <jgvicto@gmail.com> based on RemoveUnusedAccounts.php by Rob Church <robchur@gmail.com>
 */

require_once __DIR__ . '/Maintenance.php';

/**
 * Maintenance script that removes unused user accounts from the database.
 *
 * @ingroup Maintenance
 */
class DisableInactiveAccounts extends Maintenance {
	public function __construct() {
		parent::__construct();
		$this->addOption( 'disable', 'Actually disable the account' );
		$this->addOption( 'ignore-groups', 'List of comma-separated groups to exclude', false, true );
		$this->addOption( 'ignore-touched', 'Skip accounts touched in last N days', false, true );
	}

	public function execute() {

		$this->output( "Remove unused accounts\n\n" );

		# Do an initial scan for inactive accounts and report the result
		$this->output( "Checking for unused user accounts...\n" );
		$del = array();
		$dbr = wfGetDB( DB_SLAVE );
		$res = $dbr->select( 'user', array( 'user_id', 'user_name', 'user_touched' ), '', __METHOD__ );
		if ( $this->hasOption( 'ignore-groups' ) ) {
			$excludedGroups = explode( ',', $this->getOption( 'ignore-groups' ) );
		} else {
			$excludedGroups = array();
		}
		$touched = $this->getOption( 'ignore-touched', "1" );
		if ( !ctype_digit( $touched ) ) {
			$this->error( "Please put a valid positive integer on the --ignore-touched parameter.", true );
		}
		$touchedSeconds = 86400 * $touched;
		foreach ( $res as $row ) {
			# Check the account, but ignore it if it's within a $excludedGroups
			# group or if it's touched within the $touchedSeconds seconds.
			$instance = User::newFromId( $row->user_id );
			if ( count( array_intersect( $instance->getEffectiveGroups(), $excludedGroups ) ) == 0
				&& wfTimestamp( TS_UNIX, $row->user_touched ) < wfTimestamp( TS_UNIX, time() - $touchedSeconds )
			) {
				# Inactive; print out the name and flag it
				$del[] = $row->user_id;
				$this->output( $row->user_name . "\n" );
			}
		}
		$count = count( $del );
		$this->output( "...found {$count}.\n" );

		# If required, go back and disable each marked account
		if ( $count > 0 && $this->hasOption( 'disable' ) ) {
			$this->output( "\nDisabling unused accounts..." );
			$dbw = wfGetDB( DB_MASTER );
			$dbw->update( /* TABLE */ 'user',
				/* SET */ array( 'user_email'  => null ),
				/* WHERE */ array( 'user_id' => $del ),
				__METHOD__
			);
			$dbw->update( /* TABLE */ 'user',
				/* SET */ array( 'user_password'  => null ),
				/* WHERE */ array( 'user_id' => $del ),
				__METHOD__
			);
			foreach( $del as $value ) {
				$dbw->insert( /* TABLE */ 'user_groups',
					array( 'ug_user'  => $value, 'ug_group' => 'inactive' ),
					__METHOD__
				);
			}
		} elseif ( $count > 0 ) {
			$this->output( "\nRun the script again with --disable to disable them in the database.\n" );
		}
		$this->output( "\n" );
	}

}

$maintClass = "DisableInactiveAccounts";
require_once RUN_MAINTENANCE_IF_MAIN;

